#!/usr/bin/env python3
"""duo_scene.py — the transfer cell for two robots: the G1 at pallet A, the Moz1 at pallet B.

    duo_scene.py --out-dir /tmp/hmlv_duo      # -> hmlv_duo.world, hmlv_duo.yaml

The same cell as transfer_scene.py (laid out for the G1: robots/g1.yaml's scene —
pallet A 4 layers, the pair from row 4; pallet B with the open box on its deck, the
Moz1's verified placement), run as a line in steady state. The belt stops at three
stations, one step (1.1 m) apart, and starts with four cans on it:

    LOAD    y 1.3   empty   the G1 sets the pair it picks from pallet A down here
    FILLER  y 2.4   2 empty cans, waiting under the filling station (moved here from
                    the unload end: the middle of the belt)
    UNLOAD  y 3.5   2 FILLED cans (4.2 kg, red), for the Moz1

One cycle (scripts/duo_demo.py): the G1 moves a pair pallet A -> LOAD while the Moz1
moves the filled pair UNLOAD -> the box on pallet B; then the belt (IndexingConveyor,
src/indexing_conveyor.cc) fills the pair under the filler and indexes everything one
station: filled pair to UNLOAD, the G1's pair to FILLER — the start again.
"""
import argparse
import os
import re
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import transfer_scene as ts  # noqa: E402
from transfer_scene import _can_geometry, _f  # noqa: E402

sys.path.insert(0, os.path.join(get_package_prefix("moz1_sim_gazebo"), "lib", "moz1_sim_gazebo"))
from jerrycan_scene import CAN_RGBA  # noqa: E402

STATIONS = {"A": 0.0, "load": 1.30, "fill": 2.40, "unload": 3.50, "B": 5.30}
STEP = STATIONS["fill"] - STATIONS["load"]      # = unload - fill: one index of the belt
FILL_TIME = 3.0                                 # s under the filler


def build(g1):
    sc = g1["scene"]
    s = ts.build_scene(sc["cleared_rows"], sc["layers"], sc["base_front"], sc["base_z"],
                       "hmlv_duo", box_on_floor=False)
    s["stations"] = {k: [0.0, v] for k, v in STATIONS.items()}
    s["step"] = STEP
    s["travel"] = STEP
    cv, off = s["conveyor"], s["belt_offset"]
    belt_x = (cv["x_min"] + cv["x_max"]) / 2
    # the four cans on the belt: side = the hand that picks it (left = +y)
    s["belt_cans"] = [
        {"name": f"jerrycan_{tag}{i}", "pos": [belt_x, STATIONS[st] + (off if side == "left"
                                                                        else -off), cv["top"]],
         "side": side, "filled": filled, "station": st}
        for tag, st, filled in (("f", "unload", True), ("e", "fill", False))
        for i, side in enumerate(("left", "right"))]
    # which robot welds which can (one KinematicGrasp per robot-side-can): the G1 the
    # pallet-A candidates, the Moz1 whatever stands at the unload station (now the
    # filled pair, after one index the pair from the filler)
    s["grasp_targets"] = {
        "g1": [{"name": t["name"], "side": t["side"]} for t in s["targets"]],
        "moz1": [{"name": c["name"], "side": c["side"]} for c in s["belt_cans"]]}
    return s


def world_sdf(s):
    # transfer_scene's world, with the filling station over the middle of the belt ...
    sdf = ts.world_sdf(dict(s, stations=dict(s["stations"], unload=s["stations"]["fill"])))
    cv = s["conveyor"]
    plugin = f"""<plugin filename="hmlv_indexing_conveyor" name="hmlv_cell_gazebo::IndexingConveyor">
      <model_prefix>jerrycan_</model_prefix>
      <min>{_f(cv['x_min'], cv['y_min'], cv['top'] - 0.03)}</min>
      <max>{_f(cv['x_max'], cv['y_max'], cv['top'] + 0.05)}</max>
      <fill_min>{_f(cv['x_min'], STATIONS['fill'] - 0.45, cv['top'] - 0.03)}</fill_min>
      <fill_max>{_f(cv['x_max'], STATIONS['fill'] + 0.45, cv['top'] + 0.05)}</fill_max>
      <direction>0 1 0</direction>
      <speed>0.25</speed>
      <step>{STEP}</step>
      <fill_mass>{s['filled_mass']}</fill_mass>
      <fill_time>{FILL_TIME}</fill_time>
      <fill_rgba>{s['filled_rgba']}</fill_rgba>
      <start_topic>/conveyor/start</start_topic>
      <done_topic>/conveyor/done</done_topic>
    </plugin>"""
    # ... and the indexing belt instead of the carry-to-the-end one
    sdf, n = re.subn(r'<plugin filename="moz1_conveyor_belt".*?</plugin>', plugin, sdf,
                     flags=re.S)
    assert n == 1, "conveyor plugin not found in transfer_scene's world"
    c, h = s["container"], s["container"]["h"]
    W, L = c["W"], c["L"]
    models = []
    for can in s["belt_cans"]:
        m = s["filled_mass"] if can["filled"] else s["empty_mass"]
        ixx, iyy, izz = m / 12 * (L ** 2 + h ** 2), m / 12 * (W ** 2 + h ** 2), m / 12 * (W ** 2 + L ** 2)
        geo = _can_geometry(s, 0, 0, 0, prefix="")
        if can["filled"]:
            geo = geo.replace(" ".join(CAN_RGBA.split()), s["filled_rgba"])
        models.append(f"""
    <model name="{can['name']}"><pose>{_f(*can['pos'])} 0 0 0</pose>
      <link name="link">
        <inertial><pose>{_f(0, 0, h / 2)} 0 0 0</pose><mass>{m}</mass>
          <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        {geo}
      </link>
    </model>""")
    sdf = sdf.replace("  </world>", "".join(models) + "\n  </world>")
    return sdf.replace("GENERATED by hmlv_cell_gazebo/scripts/transfer_scene.py",
                       "GENERATED by hmlv_cell_gazebo/scripts/duo_scene.py")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out-dir", default=".")
    a = ap.parse_args()
    with open(os.path.join(get_package_share_directory("hmlv_cell_gazebo"), "robots",
                           "g1.yaml")) as f:
        g1 = yaml.safe_load(f)
    s = build(g1)
    os.makedirs(a.out_dir, exist_ok=True)
    world = os.path.join(a.out_dir, "hmlv_duo.world")
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
    scene_yaml = os.path.join(a.out_dir, "hmlv_duo.yaml")
    with open(scene_yaml, "w") as f:
        f.write("# GENERATED by duo_scene.py — the duo demo's view of the world file.\n")
        yaml.safe_dump(rnd({k: v for k, v in s.items() if k != "static_cans"}), f,
                       sort_keys=False, default_flow_style=None)
    print(f"{world}\n{scene_yaml}\npallet A: {len(s['static_cans'])} static + "
          f"{len(s['targets'])} candidates; belt: 2 filled at unload, 2 empty at the filler")


if __name__ == "__main__":
    main()
