#!/usr/bin/env python3
"""jerrycan_scene.py — generate the jerry-can depalletising world for the Moz1.

Writes a gz-sim world (SDF) and a matching scene YAML for scripts/jerrycan_demo.py:

    jerrycan_scene.py --container 4 --layers 3 --out-dir /tmp/moz1_jerrycan

The scene is the HMLV liquid-filling-line depalletising cell from the humanoid
pallet-reach check (humanoid-reach/index.html): a 1200 x 1000 mm pallet of HDPE
jerry cans, worked by the robot from its 1200 mm side with a 50 mm base-to-pallet
gap, plus one infeed conveyor on each side of the robot to place the cans on.

Container data are the TCCC figures from that page (dimensions and UN marking
published; empty weights are estimates there too). The can's short side (W) runs
along the pallet depth, so rows = floor(depth / W). Cans are stacked handle-up with
no separators, grasp point = the handle on top.

Frames: everything is in the gz WORLD frame. The robot is spawned with a fixed base
(sim_gazebo.launch.py fix_base:=true), which puts base_link at (0, 0, BASE_Z)
facing +x. The YAML records BASE_Z so the demo can express poses in base_link.

Only the top layer's front row (nearest the robot) is dynamic and graspable; the
rest of the stack is one static model, which keeps physics cheap. Left-arm targets
are the cans at y > 0, right-arm targets the cans at y < 0.
"""
import argparse
import os

import yaml

BASE_Z = 0.11            # base_link height with fix_base (see sim_gazebo.launch.py)
BASE_FRONT = 0.321       # chassis front face, x (from the base_link.STL bounds)

# TCCC containers, from humanoid-reach/index.html CONT_DEFAULTS (mm, g, L).
CONTAINERS = {
    "4":  {"label": "4 L",  "vol": 4.0,  "h": 240, "L": 180, "W": 120, "empty": 200,
           "un": "3H1/Y1.4/100/26", "max_sg": 1.4},
    "10": {"label": "10 L", "vol": 10.0, "h": 240, "L": 270, "W": 220, "empty": 550,
           "un": "3H1/Y1.8/200/26", "max_sg": 1.8},
    "20": {"label": "20 L", "vol": 20.0, "h": 420, "L": 270, "W": 215, "empty": 1050,
           "un": "3H1/Y1.4/100/26", "max_sg": 1.4},
}
DEFAULT_LAYERS = {"4": 3, "10": 3, "20": 2}

PALLET = {"length": 1.2, "depth": 1.0, "deck": 0.15, "gap": 0.05}
CAN_GAP = 0.004          # between neighbouring cans (keeps dynamic cans from jittering)
HANDLE_H = 0.045         # handle zone on top of the body: posts + bar
BAR_T = 0.024            # bar thickness across the jaw (x)
BAR_H = 0.022            # bar height
CONVEYOR = {"top": 0.75, "width": 0.34, "y": 0.68, "x_min": -0.95, "x_max": 0.30}
# Moz1 rated payload per arm is NOT published; 5 kg is the reach page's placeholder.
PAYLOAD_PER_ARM = 5.0

CAN_RGBA = "0.10 0.32 0.72 1"
CAP_RGBA = "0.95 0.75 0.10 1"


def build_scene(container="4", layers=None, density=1.0, step=0.004):
    c = CONTAINERS[container]
    layers = layers or DEFAULT_LAYERS[container]
    h, L, W = c["h"] / 1000, c["L"] / 1000, c["W"] / 1000
    body_h = h - HANDLE_H
    bar_len = min(0.11, L - 0.09)
    handle_y = -L / 2 + 0.03 + bar_len / 2    # handle at one end, spout at the other
    grasp_z = h - BAR_H / 2                   # bar centre above the can bottom
    mass = density * c["vol"] + c["empty"] / 1000

    x0 = BASE_FRONT + PALLET["gap"]           # pallet near edge
    cols = int(PALLET["length"] // (L + CAN_GAP))
    rows = int(PALLET["depth"] // (W + CAN_GAP))
    deck = PALLET["deck"]

    def col_y(ci):
        return (ci - (cols - 1) / 2) * (L + CAN_GAP)

    def row_x(ri):
        return x0 + CAN_GAP / 2 + W / 2 + ri * (W + CAN_GAP)

    static_cans, targets = [], []
    for k in range(layers):
        for ri in range(rows):
            for ci in range(cols):
                pos = [row_x(ri), col_y(ci), deck + k * h]   # can bottom centre
                if k == layers - 1 and ri == 0:
                    y = pos[1]
                    targets.append({"pos": pos, "side": "left" if y > 0 else "right",
                                    "col": ci})
                else:
                    static_cans.append(pos)
    # Pick order: alternate arms, inner cans first (short carries first).
    by_side = {s: sorted([t for t in targets if t["side"] == s], key=lambda t: abs(t["pos"][1]))
               for s in ("left", "right")}
    ordered = []
    for i in range(max(len(v) for v in by_side.values())):
        for s in ("left", "right"):
            if i < len(by_side[s]):
                ordered.append(by_side[s][i])
    for t in ordered:
        n = sum(1 for o in ordered[:ordered.index(t) + 1] if o["side"] == t["side"])
        t["name"] = f"jerrycan_{t['side']}_{n}"

    # Place slots on each side's conveyor, filled BACK to front: the arm arrives
    # from the pallet (+x), so it never has to reach over a can it already placed.
    slots = {}
    for side, sgn in (("left", 1), ("right", -1)):
        n = len(by_side[side])
        xs = [0.15 - i * (W + 0.10) for i in reversed(range(n))]
        slots[side] = [[x, sgn * CONVEYOR["y"], CONVEYOR["top"]] for x in xs]

    stack_top = deck + layers * h
    boxes = [
        # pallet
        {"name": "pallet", "center": [x0 + PALLET["depth"] / 2, 0.0, deck / 2],
         "size": [PALLET["depth"], PALLET["length"], deck]},
        # static stack behind the front row, up to the top layer's can BODIES; the
        # handles are separate boxes below. A slab up to the handle tops blocks the
        # open jaw, which reaches past the front row's far face on the way down.
        {"name": "stack_back",
         "center": [(row_x(1) - W / 2 + row_x(rows - 1) + W / 2) / 2, 0.0,
                    (deck + stack_top - HANDLE_H) / 2],
         "size": [row_x(rows - 1) - row_x(1) + W, cols * (L + CAN_GAP),
                  stack_top - HANDLE_H - deck]},
    ]
    for ri in range(1, rows):
        for ci in range(cols):
            boxes.append({"name": f"stack_handle_{ri}_{ci}",
                          "center": [row_x(ri), col_y(ci) + handle_y, stack_top - HANDLE_H / 2],
                          "size": [BAR_T, bar_len, HANDLE_H]})
    if layers > 1:
        top_front = deck + (layers - 1) * h
        boxes.append({"name": "stack_front_lower",
                      "center": [row_x(0), 0.0, (deck + top_front) / 2],
                      "size": [W, cols * (L + CAN_GAP), top_front - deck]})
    for side, sgn in (("left", 1), ("right", -1)):
        boxes.append({"name": f"conveyor_{side}",
                      "center": [(CONVEYOR["x_min"] + CONVEYOR["x_max"]) / 2,
                                 sgn * CONVEYOR["y"], CONVEYOR["top"] / 2],
                      "size": [CONVEYOR["x_max"] - CONVEYOR["x_min"], CONVEYOR["width"],
                               CONVEYOR["top"]]})

    return {
        "container": dict(c, key=container, h=h, L=L, W=W),
        "density": density,
        "mass": round(mass, 3),
        "payload_per_arm": PAYLOAD_PER_ARM,
        "layers": layers, "rows": rows, "cols": cols,
        "world_name": "moz1_jerrycan",
        "physics_step": step,
        "base_z": BASE_Z,
        "can": {"body_h": body_h, "handle_y": handle_y, "grasp_z": grasp_z,
                "bar_len": bar_len},
        "targets": [{"name": t["name"], "side": t["side"], "pos": t["pos"]} for t in ordered],
        "place_slots": slots,
        "static_cans": static_cans,
        "collision_boxes": boxes,
        "stack_top": stack_top,
        "conveyor": CONVEYOR,
        "pallet": dict(PALLET, x0=x0),
    }


# --------------------------------------------------------------------------- SDF
def _f(*v):
    return " ".join(f"{x:.4f}" for x in v)


def _box_geom(sx, sy, sz):
    return f"<geometry><box><size>{_f(sx, sy, sz)}</size></box></geometry>"


def _mat(rgba):
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def _can_geometry(scene, ox, oy, oz, prefix, stacked=False):
    """Visual + collision elements of one can whose bottom centre is (ox, oy, oz)
    in the enclosing link frame.
      * stacked (static stack): ONE full-height collision box, W x L x h. Jerry
        cans stack on their full height (pitch = h), so the can above must rest at
        h, not on the body top 45 mm lower with the handle sunk into it — which is
        what a body-only collision did, silently dropping every pick target 45 mm.
      * pick target: body collision only, so the open jaw can straddle the bar.
        The handle is visual; the grasp itself is the KinematicGrasp hold."""
    c, g = scene["container"], scene["can"]
    W, L, bh = c["W"], c["L"], g["body_h"]
    hy, bl = g["handle_y"], g["bar_len"]
    post_h = HANDLE_H - BAR_H
    parts = [
        ("body", (ox, oy, oz + bh / 2), (W, L, bh), CAN_RGBA, not stacked),
        ("bar", (ox, oy + hy, oz + c["h"] - BAR_H / 2), (BAR_T, bl, BAR_H), CAN_RGBA, False),
        ("post_a", (ox, oy + hy - bl / 2 + 0.01, oz + bh + post_h / 2), (BAR_T, 0.02, post_h),
         CAN_RGBA, False),
        ("post_b", (ox, oy + hy + bl / 2 - 0.01, oz + bh + post_h / 2), (BAR_T, 0.02, post_h),
         CAN_RGBA, False),
    ]
    out = []
    if stacked:
        out.append(f'<collision name="{prefix}envelope_c"><pose>{_f(ox, oy, oz + c["h"] / 2)} '
                   f'0 0 0</pose>{_box_geom(W, L, c["h"])}</collision>')
    for name, pos, size, rgba, col in parts:
        pose = f"<pose>{_f(*pos)} 0 0 0</pose>"
        out.append(f'<visual name="{prefix}{name}">{pose}{_box_geom(*size)}{_mat(rgba)}</visual>')
        if col:
            out.append(f'<collision name="{prefix}{name}_c">{pose}{_box_geom(*size)}'
                       f"<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>"
                       f"</surface></collision>")
    cap_y = oy + L / 2 - 0.03
    out.append(f'<visual name="{prefix}cap"><pose>{_f(ox, cap_y, oz + bh + 0.012)} 0 0 0</pose>'
               f"<geometry><cylinder><radius>0.021</radius><length>0.024</length></cylinder>"
               f"</geometry>{_mat(CAP_RGBA)}</visual>")
    return "\n        ".join(out)


def pallet_model(name, x0, y0, p):
    """EUR pallet (deck boards, bottom boards, 9 blocks; one static collision box)
    whose near edge is at x0, centred on y0, the 1200 mm side facing -x."""
    D, Lp, deck = p["depth"], p["length"], p["deck"]
    wood = "0.62 0.48 0.30 1"
    pv = []
    for i, yy in enumerate([-0.55, -0.35, -0.12, 0.12, 0.35, 0.55]):
        pv.append(f'<visual name="deck{i}"><pose>{_f(D / 2, yy, deck - 0.011)} 0 0 0</pose>'
                  f"{_box_geom(D, 0.10 if abs(yy) > 0.5 else 0.14, 0.022)}{_mat(wood)}</visual>")
    for i, xx in enumerate([0.05, D / 2, D - 0.05]):
        pv.append(f'<visual name="bottom{i}"><pose>{_f(xx, 0, 0.011)} 0 0 0</pose>'
                  f"{_box_geom(0.10, Lp, 0.022)}{_mat(wood)}</visual>")
        for j, yy in enumerate([-0.55, 0.0, 0.55]):
            pv.append(f'<visual name="block{i}{j}"><pose>{_f(xx, yy, deck / 2)} 0 0 0</pose>'
                      f"{_box_geom(0.10, 0.10 if abs(yy) > 0.5 else 0.145, deck - 0.044)}"
                      f"{_mat(wood)}</visual>")
    return f"""
    <!-- EUR pallet 1200 x 1000 x 150 mm; robot works from the 1200 mm side -->
    <model name="{name}"><static>true</static><pose>{_f(x0, y0, 0)} 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>{_f(D / 2, 0, deck / 2)} 0 0 0</pose>{_box_geom(D, Lp, deck)}</collision>
        {chr(10).join(pv)}
      </link>
    </model>"""


def world_sdf(scene):
    c, p, cv = scene["container"], scene["pallet"], scene["conveyor"]
    m = scene["mass"]
    W, L, h = c["W"], c["L"], c["h"]
    ixx = m / 12 * (L ** 2 + h ** 2)
    iyy = m / 12 * (W ** 2 + h ** 2)
    izz = m / 12 * (W ** 2 + L ** 2)

    models = []
    models.append(pallet_model("pallet", p["x0"], 0.0, p))

    # static stack: one model, one set of boxes per can
    geo = "\n        ".join(_can_geometry(scene, *pos, prefix=f"c{i}_", stacked=True)
                           for i, pos in enumerate(scene["static_cans"]))
    models.append(f"""
    <!-- {c['label']} jerry cans ({c['un']}), {scene['layers']} layer(s) x {scene['rows']} rows x
         {scene['cols']} cols, static apart from the top layer's front row -->
    <model name="jerrycan_stack"><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="link">
        {geo}
      </link>
    </model>""")

    # dynamic pick targets
    for t in scene["targets"]:
        x, y, z = t["pos"]
        models.append(f"""
    <model name="{t['name']}"><pose>{_f(x, y, z)} 0 0 0</pose>
      <link name="link">
        <inertial><pose>{_f(0, 0, h / 2)} 0 0 0</pose><mass>{m:.3f}</mass>
          <inertia><ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        {_can_geometry(scene, 0, 0, 0, prefix="")}
      </link>
    </model>""")

    # conveyors
    frame = "0.35 0.37 0.40 1"
    belt = "0.08 0.08 0.09 1"
    for side, sgn in (("left", 1), ("right", -1)):
        cx = (cv["x_min"] + cv["x_max"]) / 2
        ln = cv["x_max"] - cv["x_min"]
        wd, top = cv["width"], cv["top"]
        legs = "\n        ".join(
            f'<visual name="leg{i}{j}"><pose>{_f(lx, ly, (top - 0.06) / 2)} 0 0 0</pose>'
            f"{_box_geom(0.04, 0.04, top - 0.06)}{_mat(frame)}</visual>"
            for i, lx in enumerate([-ln / 2 + 0.05, 0, ln / 2 - 0.05])
            for j, ly in enumerate([-wd / 2 + 0.03, wd / 2 - 0.03]))
        rollers = "\n        ".join(
            f'<visual name="roller{i}"><pose>{_f(-ln / 2 + 0.04 + i * 0.08, 0, top - 0.035)} '
            f"1.5708 0 0</pose><geometry><cylinder><radius>0.02</radius><length>{wd - 0.02:.3f}"
            f"</length></cylinder></geometry>{_mat('0.7 0.7 0.72 1')}</visual>"
            for i in range(int((ln - 0.08) // 0.08) + 1))
        models.append(f"""
    <!-- {side} infeed conveyor to the filling line, belt top {top:.2f} m -->
    <model name="conveyor_{side}"><static>true</static><pose>{_f(cx, sgn * cv['y'], 0)} 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>{_f(0, 0, top / 2)} 0 0 0</pose>{_box_geom(ln, wd, top)}</collision>
        <visual name="belt"><pose>{_f(0, 0, top - 0.005)} 0 0 0</pose>{_box_geom(ln, wd - 0.04, 0.01)}{_mat(belt)}</visual>
        <visual name="rail_a"><pose>{_f(0, wd / 2 - 0.01, top - 0.03)} 0 0 0</pose>{_box_geom(ln, 0.02, 0.07)}{_mat(frame)}</visual>
        <visual name="rail_b"><pose>{_f(0, -wd / 2 + 0.01, top - 0.03)} 0 0 0</pose>{_box_geom(ln, 0.02, 0.07)}{_mat(frame)}</visual>
        {rollers}
        {legs}
      </link>
    </model>""")

    return f"""<?xml version="1.0"?>
<!-- GENERATED by moz1_sim_gazebo/scripts/jerrycan_scene.py — edit that, not this.

     Jerry-can depalletising cell for the Moz1 (Gazebo Fortress): a EUR pallet of
     {c['label']} HDPE jerry cans ({c['un']}, {W * 1000:.0f} x {L * 1000:.0f} x {h * 1000:.0f} mm,
     {m:.2f} kg filled at density {scene['density']:.2f}) in front of the robot, and an
     infeed conveyor on each side. Robot spawns pinned at the origin facing +x.
     The {len(scene['targets'])} top-front cans are the dynamic pick targets. -->
<sdf version="1.8">
  <world name="moz1_jerrycan">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <physics name="step" type="ignored">
      <max_step_size>{scene['physics_step']}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><grid>false</grid></scene>

    <gui fullscreen="0">
      <camera name="user_camera"><pose>2.6 -2.2 2.1 0 0.42 2.45</pose></camera>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -1</direction>
    </light>

    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>
{"".join(models)}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--container", default="4", choices=sorted(CONTAINERS, key=float),
                    help="jerry can size in litres (TCCC 4 / 10 / 20 L)")
    ap.add_argument("--layers", type=int, default=0,
                    help="stack levels (default: 3 for 4/10 L, 2 for 20 L)")
    ap.add_argument("--density", type=float, default=1.0, help="liquid density, g/mL")
    ap.add_argument("--step", type=float, default=0.004, help="physics step, s")
    ap.add_argument("--out-dir", default=".", help="where to write the .world and .yaml")
    ap.add_argument("--name", default="moz1_jerrycan", help="output file stem")
    a = ap.parse_args()

    scene = build_scene(a.container, a.layers or None, a.density, a.step)
    os.makedirs(a.out_dir, exist_ok=True)
    world = os.path.join(a.out_dir, f"{a.name}.world")
    with open(world, "w") as f:
        f.write(world_sdf(scene))
    def rnd(v):
        if isinstance(v, float):
            return round(v, 4)
        if isinstance(v, list):
            return [rnd(x) for x in v]
        if isinstance(v, dict):
            return {k: rnd(x) for k, x in v.items()}
        return v
    scene_yaml = os.path.join(a.out_dir, f"{a.name}.yaml")
    with open(scene_yaml, "w") as f:
        f.write("# GENERATED by jerrycan_scene.py — the demo's view of the world file.\n")
        yaml.safe_dump(rnd({k: v for k, v in scene.items() if k != "static_cans"}), f,
                       sort_keys=False, default_flow_style=None)
    c = scene["container"]
    over = scene["mass"] > scene["payload_per_arm"]
    print(f"{world}\n{scene_yaml}\n{c['label']}: {scene['layers']} layers x {scene['rows']} rows"
          f" x {scene['cols']} cols, {len(scene['targets'])} targets, {scene['mass']:.2f} kg"
          f" filled{' — OVER the 5 kg per-arm placeholder rating' if over else ''}")
    if scene["density"] > c["max_sg"]:
        print(f"warning: density {scene['density']} exceeds the UN rating {c['un']}")


if __name__ == "__main__":
    main()
