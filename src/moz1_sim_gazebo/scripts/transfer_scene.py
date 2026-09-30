#!/usr/bin/env python3
"""transfer_scene.py — the jerry-can filling-line transfer cell for the Moz1.

Writes a gz-sim world (SDF) and a scene YAML for scripts/transfer_demo.py:

    transfer_scene.py --out-dir /tmp/moz1_transfer

Layout (gz world frame; the robot starts at the origin facing +x, base FREE and
driving along y, always in front of the pallets and the conveyor):

    pallet A   y ≈ 0      4 L jerry cans, EMPTY (0.2 kg), 3 layers (--layers). The top
                          layer's first --cleared-rows rows are already gone, so
                          the next row can only be reached by leaning the torso
                          over the pallet.
    conveyor   y 0.85…4.05, belt 0.65 m, running +y. Cans put on it at the LOAD
                          station are carried 2.2 m to the UNLOAD station and
                          filled to 4.2 kg on the way (ConveyorBelt plugin,
                          src/conveyor_belt.cc).
    pallet B   y ≈ 5.3    an open cardboard box on its deck, nearest row; the
                          filled pair goes into it, side by side. The box sits
                          on the deck (not the floor) so the deck place poses
                          still hold: the cans end up one box floor higher.

Only the top layer's first remaining row of pallet A is dynamic (the pick
candidates); everything else is static. 4 L cans only, TCCC dimensions from the
humanoid pallet-reach check (see jerrycan_scene.py).
"""
import argparse
import math
import os

import yaml

from jerrycan_scene import (BAR_H, BASE_FRONT, CAN_GAP, CAP_RGBA, CONTAINERS, HANDLE_H,
                            PALLET, _box_geom, _can_geometry, _f, _mat, pallet_model)

LAYERS = 3                 # stack height on pallet A
EMPTY_MASS = 0.2           # TCCC 4 L can, empty (estimate on the reach page)
FILLED_MASS = 4.2          # + 4 L of water
CONVEYOR = {"x_min": 0.37, "x_max": 0.71, "y_min": 0.85, "y_max": 4.05, "top": 0.65}
STATIONS = {"A": 0.0, "load": 1.30, "unload": 3.50, "B": 5.30}   # robot base y, x = 0
BELT_OFFSET = 0.19         # can centres at station y ± this on the belt
PALLET_B_Y = STATIONS["B"]
FILLED_RGBA = "0.80 0.10 0.10 1"   # the conveyor recolours a can when it fills it
# Open cardboard box on pallet B for the filled pair. Walls stay well below the
# grasp (bar at 229 mm) and clear the can bottom at the pre-place height.
BOX = {"margin": 0.025,    # can to inner wall, each side (placement slack)
       "wall": 0.005, "floor": 0.005, "wall_h": 0.10, "flap": 0.08,
       "shift": 0.03}      # slots moved away from the robot so the box's near
                           # wall sits on the pallet's edge (0 = old slots)
CARDBOARD_RGBA = "0.80 0.64 0.42 1"


def build_scene(cleared_rows=2, layers=LAYERS):
    c = CONTAINERS["4"]
    h, L, W = c["h"] / 1000, c["L"] / 1000, c["W"] / 1000
    bar_len = min(0.11, L - 0.09)
    can = {"body_h": h - HANDLE_H, "handle_y": -L / 2 + 0.03 + bar_len / 2,
           "grasp_z": h - BAR_H / 2, "bar_len": bar_len}
    x0 = BASE_FRONT + PALLET["gap"]
    deck = PALLET["deck"]
    cols = int(PALLET["length"] // (L + CAN_GAP))
    rows = int(PALLET["depth"] // (W + CAN_GAP))
    if not 0 <= cleared_rows < rows:
        raise ValueError(f"--cleared-rows must be 0..{rows - 1}")

    def col_y(ci, y0=0.0):
        return y0 + (ci - (cols - 1) / 2) * (L + CAN_GAP)

    def row_x(ri):
        return x0 + CAN_GAP / 2 + W / 2 + ri * (W + CAN_GAP)

    top = layers - 1
    static_cans, targets = [], []
    for k in range(layers):
        for ri in range(rows):
            if k == top and ri < cleared_rows:
                continue                                   # already taken
            for ci in range(cols):
                pos = [row_x(ri), col_y(ci), deck + k * h]
                if k == top and ri == cleared_rows:
                    targets.append({"name": f"jerrycan_a{ci}", "pos": pos,
                                    "side": "left" if pos[1] > 0 else "right"})
                else:
                    static_cans.append(pos)

    stack_top = deck + layers * h
    lower_top = deck + (layers - 1) * h
    cv = CONVEYOR
    boxes = [
        {"name": "pallet_a", "center": [x0 + PALLET["depth"] / 2, 0.0, deck / 2],
         "size": [PALLET["depth"], PALLET["length"], deck]},
        {"name": "stack_lower", "center": [x0 + PALLET["depth"] / 2, 0.0, (deck + lower_top) / 2],
         "size": [rows * (W + CAN_GAP), cols * (L + CAN_GAP), lower_top - deck]},
        {"name": "conveyor", "center": [(cv["x_min"] + cv["x_max"]) / 2,
                                        (cv["y_min"] + cv["y_max"]) / 2, cv["top"] / 2],
         "size": [cv["x_max"] - cv["x_min"], cv["y_max"] - cv["y_min"], cv["top"]]},
        {"name": "pallet_b", "center": [x0 + PALLET["depth"] / 2, PALLET_B_Y, deck / 2],
         "size": [PALLET["depth"], PALLET["length"], deck]},
    ]
    # the box: around the two middle slots of pallet B's nearest row
    slot_x = row_x(0) + BOX["shift"]
    slot_y = {"left": col_y(cols // 2, PALLET_B_Y), "right": col_y(cols // 2 - 1, PALLET_B_Y)}
    t, fl, wh, m = BOX["wall"], BOX["floor"], BOX["wall_h"], BOX["margin"]
    inner = [W + 2 * m, abs(slot_y["left"] - slot_y["right"]) + L + 2 * m]
    box = {"center": [slot_x, (slot_y["left"] + slot_y["right"]) / 2, deck],   # bottom centre
           "inner": inner, "wall": t, "floor": fl, "wall_h": wh, "flap": BOX["flap"]}
    boxes += _box_parts(box)
    if cleared_rows + 1 < rows:     # the top layer behind the candidate row, handles included
        x_back0 = row_x(cleared_rows + 1) - W / 2
        x_back1 = row_x(rows - 1) + W / 2
        boxes.append({"name": "stack_top_back",
                      "center": [(x_back0 + x_back1) / 2, 0.0, (lower_top + stack_top) / 2],
                      "size": [x_back1 - x_back0, cols * (L + CAN_GAP), stack_top - lower_top]})

    return {
        "world_name": "moz1_transfer",
        "base_z": 0.105,          # free base: base_link height on its wheels
        "container": dict(c, key="4", h=h, L=L, W=W),
        "can": can,
        "empty_mass": EMPTY_MASS, "filled_mass": FILLED_MASS,
        "layers": layers, "rows": rows, "cols": cols, "cleared_rows": cleared_rows,
        "stack_top": stack_top,
        "stations": {k: [0.0, v] for k, v in STATIONS.items()},
        "belt_offset": BELT_OFFSET,
        "conveyor": cv,
        "travel": STATIONS["unload"] - STATIONS["load"],
        # pallet B: inside the box on the deck (can bottoms on the box floor)
        "b_slots": {s_: [slot_x, slot_y[s_], deck + fl] for s_ in ("left", "right")},
        "box_b": box,
        "filled_rgba": FILLED_RGBA,
        "targets": targets,
        "static_cans": static_cans,
        "collision_boxes": boxes,
        "pallet": dict(PALLET, x0=x0),
    }


def _box_parts(b):
    """The open box's floor and four walls as {name, center, size} boxes (gz)."""
    (cx, cy, z0), (ix, iy) = b["center"], b["inner"]
    t, fl, wh = b["wall"], b["floor"], b["wall_h"]
    ox, oy, top = ix + 2 * t, iy + 2 * t, z0 + fl + wh
    zc = (z0 + fl + top) / 2
    return [
        {"name": "box_b_floor", "center": [cx, cy, z0 + fl / 2], "size": [ox, oy, fl]},
        {"name": "box_b_near", "center": [cx - (ix + t) / 2, cy, zc], "size": [t, oy, wh]},
        {"name": "box_b_far", "center": [cx + (ix + t) / 2, cy, zc], "size": [t, oy, wh]},
        {"name": "box_b_left", "center": [cx, cy + (iy + t) / 2, zc], "size": [ix, t, wh]},
        {"name": "box_b_right", "center": [cx, cy - (iy + t) / 2, zc], "size": [ix, t, wh]},
    ]


def box_model(b):
    """Static open cardboard box: floor and walls collide; the four flaps (visual
    only) hang folded back outside the walls, clear of the hands."""
    parts = []
    for p in _box_parts(b):
        pose = f"<pose>{_f(*p['center'])} 0 0 0</pose>"
        n = p["name"]
        parts.append(f'<collision name="{n}_c">{pose}{_box_geom(*p["size"])}</collision>')
        parts.append(f'<visual name="{n}">{pose}{_box_geom(*p["size"])}{_mat(CARDBOARD_RGBA)}</visual>')
    (cx, cy, z0), (ix, iy) = b["center"], b["inner"]
    t, fl, top, fx = b["wall"], b["floor"], z0 + b["floor"] + b["wall_h"], b["flap"]
    a = math.radians(25)                     # flap angle from hanging straight down
    dx, dz = math.sin(a) * fx / 2, math.cos(a) * fx / 2
    ex, ey = (ix + 2 * t) / 2, (iy + 2 * t) / 2
    for n, pos, rpy, size in [
            ("near", (cx - ex - dx, cy, top - dz), (0, a, 0), (t, iy + 2 * t, fx)),
            ("far", (cx + ex + dx, cy, top - dz), (0, -a, 0), (t, iy + 2 * t, fx)),
            ("left", (cx, cy + ey + dx, top - dz), (a, 0, 0), (ix + 2 * t, t, fx)),
            ("right", (cx, cy - ey - dx, top - dz), (-a, 0, 0), (ix + 2 * t, t, fx))]:
        parts.append(f'<visual name="flap_{n}"><pose>{_f(*pos, *rpy)}</pose>'
                     f"{_box_geom(*size)}{_mat(CARDBOARD_RGBA)}</visual>")
    body = "\n        ".join(parts)
    return f"""
    <!-- open cardboard box on pallet B for the filled pair -->
    <model name="box_b"><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="link">
        {body}
      </link>
    </model>"""


def world_sdf(s):
    c, cv = s["container"], s["conveyor"]
    W, L, h = c["W"], c["L"], c["h"]
    m = s["empty_mass"]
    ixx, iyy, izz = m / 12 * (L ** 2 + h ** 2), m / 12 * (W ** 2 + h ** 2), m / 12 * (W ** 2 + L ** 2)
    models = [pallet_model("pallet_a", s["pallet"]["x0"], 0.0, s["pallet"]),
              pallet_model("pallet_b", s["pallet"]["x0"], PALLET_B_Y, s["pallet"]),
              box_model(s["box_b"])]

    geo = "\n        ".join(_can_geometry(s, *pos, prefix=f"c{i}_", stacked=True)
                           for i, pos in enumerate(s["static_cans"]))
    models.append(f"""
    <model name="stack_a"><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="link">
        {geo}
      </link>
    </model>""")
    for t in s["targets"]:
        models.append(f"""
    <model name="{t['name']}"><pose>{_f(*t['pos'])} 0 0 0</pose>
      <link name="link">
        <inertial><pose>{_f(0, 0, h / 2)} 0 0 0</pose><mass>{m}</mass>
          <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        {_can_geometry(s, 0, 0, 0, prefix="")}
      </link>
    </model>""")

    # conveyor along +y
    frame, belt = "0.35 0.37 0.40 1", "0.08 0.08 0.09 1"
    cx, cy = (cv["x_min"] + cv["x_max"]) / 2, (cv["y_min"] + cv["y_max"]) / 2
    wd, ln, top = cv["x_max"] - cv["x_min"], cv["y_max"] - cv["y_min"], cv["top"]
    legs = "\n        ".join(
        f'<visual name="leg{i}{j}"><pose>{_f(lx, ly, (top - 0.06) / 2)} 0 0 0</pose>'
        f"{_box_geom(0.04, 0.04, top - 0.06)}{_mat(frame)}</visual>"
        for i, ly in enumerate([-ln / 2 + 0.05, -ln / 6, ln / 6, ln / 2 - 0.05])
        for j, lx in enumerate([-wd / 2 + 0.03, wd / 2 - 0.03]))
    rollers = "\n        ".join(
        f'<visual name="roller{i}"><pose>{_f(0, -ln / 2 + 0.04 + i * 0.08, top - 0.035)} '
        f"0 1.5708 0</pose><geometry><cylinder><radius>0.02</radius><length>{wd - 0.02:.3f}"
        f"</length></cylinder></geometry>{_mat('0.7 0.7 0.72 1')}</visual>"
        for i in range(int((ln - 0.08) // 0.08) + 1))
    fill_y = s["stations"]["unload"][1]
    models.append(f"""
    <!-- conveyor to the filling station, belt {top:.2f} m, running +y -->
    <model name="conveyor"><static>true</static><pose>{_f(cx, cy, 0)} 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>{_f(0, 0, top / 2)} 0 0 0</pose>{_box_geom(wd, ln, top)}</collision>
        <visual name="belt"><pose>{_f(0, 0, top - 0.005)} 0 0 0</pose>{_box_geom(wd - 0.04, ln, 0.01)}{_mat(belt)}</visual>
        <visual name="rail_a"><pose>{_f(wd / 2 - 0.01, 0, top - 0.03)} 0 0 0</pose>{_box_geom(0.02, ln, 0.07)}{_mat(frame)}</visual>
        <visual name="rail_b"><pose>{_f(-wd / 2 + 0.01, 0, top - 0.03)} 0 0 0</pose>{_box_geom(0.02, ln, 0.07)}{_mat(frame)}</visual>
        {rollers}
        {legs}
      </link>
    </model>
    <!-- filling head over the unload end (visual only, well above the cans) -->
    <model name="filling_station"><static>true</static><pose>{_f(cx, fill_y, 0)} 0 0 0</pose>
      <link name="link">
        <visual name="post_a"><pose>{_f(wd / 2 + 0.05, -0.30, 0.8)} 0 0 0</pose>{_box_geom(0.05, 0.05, 1.6)}{_mat(frame)}</visual>
        <visual name="post_b"><pose>{_f(wd / 2 + 0.05, 0.30, 0.8)} 0 0 0</pose>{_box_geom(0.05, 0.05, 1.6)}{_mat(frame)}</visual>
        <visual name="beam"><pose>{_f(0.1, 0, 1.55)} 0 0 0</pose>{_box_geom(wd + 0.1, 0.70, 0.10)}{_mat(CAP_RGBA)}</visual>
      </link>
    </model>""")

    load_y = s["stations"]["load"][1]
    return f"""<?xml version="1.0"?>
<!-- GENERATED by moz1_sim_gazebo/scripts/transfer_scene.py — edit that, not this.

     Jerry-can transfer cell: pallet A of EMPTY 4 L cans ({W * 1000:.0f} x {L * 1000:.0f} x
     {h * 1000:.0f} mm, {m} kg) → conveyor + filling station ({s['filled_mass']} kg, turns red) → box on pallet B.
     The robot's base is free and drives along y in front of all three. -->
<sdf version="1.8">
  <world name="{s['world_name']}">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="moz1_conveyor_belt" name="moz1_sim_gazebo::ConveyorBelt">
      <model_prefix>jerrycan_</model_prefix>
      <min>{_f(cv['x_min'], cv['y_min'], cv['top'] - 0.03)}</min>
      <max>{_f(cv['x_max'], load_y + 0.5, cv['top'] + 0.05)}</max>
      <direction>0 1 0</direction>
      <speed>0.25</speed>
      <travel>{s['travel']}</travel>
      <fill_mass>{s['filled_mass']}</fill_mass>
      <fill_rgba>{s['filled_rgba']}</fill_rgba>
      <start_topic>/conveyor/start</start_topic>
      <done_topic>/conveyor/done</done_topic>
    </plugin>

    <physics name="step" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><grid>false</grid></scene>
    <gui fullscreen="0">
      <camera name="user_camera"><pose>-3.5 2.6 3.0 0 0.5 0</pose></camera>
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
    ap.add_argument("--cleared-rows", type=int, default=2,
                    help="top-layer rows of pallet A already taken (default 2)")
    ap.add_argument("--layers", type=int, default=LAYERS,
                    help=f"stack height on pallet A; the pair comes from the top one "
                         f"(default {LAYERS})")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--name", default="moz1_transfer")
    a = ap.parse_args()
    s = build_scene(a.cleared_rows, a.layers)
    os.makedirs(a.out_dir, exist_ok=True)
    world = os.path.join(a.out_dir, f"{a.name}.world")
    with open(world, "w") as f:
        f.write(world_sdf(s))

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
        f.write("# GENERATED by transfer_scene.py — the demo's view of the world file.\n")
        yaml.safe_dump(rnd({k: v for k, v in s.items() if k != "static_cans"}), f,
                       sort_keys=False, default_flow_style=None)
    print(f"{world}\n{scene_yaml}\npallet A: {len(s['static_cans'])} static + "
          f"{len(s['targets'])} candidate cans (top-layer row {a.cleared_rows + 1})")


if __name__ == "__main__":
    main()
