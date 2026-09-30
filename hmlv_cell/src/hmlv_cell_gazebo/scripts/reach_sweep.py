#!/usr/bin/env python3
"""reach_sweep.py — which torso poses reach which pallet-A row (and the pallet-B box).

Offline check against a running move_group (the cell launched with demo:=false);
nothing moves. For each top-layer row of pallet A (the scene rebuilt with that
many rows cleared in front of it), and for the box at pallet B, every torso pose
of a grid over the leg joints plus the profile's presets is tried with the demo's
own reach test: collision-aware IK for both arms at the pre-grasp and the grasp
pose (TransferDemo.dual_ik, torso included), the innermost pair of the row —
and, since the base is free, a tipping check: the whole robot's centre of mass
at the grasp (MoveIt FK of every link's inertial, from /robot_description, plus
the pair in the hands: empty at pallet A, filled at the box) must stay
--com_max_x behind base_link's origin (the G1's front wheels touch down at
x = +0.18 m; leaned out over pallet A's row 5 it tipped 4.4° onto them and slid
6 cm back, and every straight move planned for the level base was off).

  ros2 launch hmlv_cell_gazebo transfer.launch.py robot:=g1 demo:=false rviz:=false
  ros2 run hmlv_cell_gazebo reach_sweep.py --ros-args -p robot_file:=<share>/robots/g1.yaml \\
      -p scene_file:=/tmp/hmlv_transfer_g1/hmlv_transfer.yaml -p layers:=4

Parameters: robot_file, scene_file (the launched one: base_front / base_z / world),
layers (default: the scene's), rows ("" = all, else e.g. "3,4,5"), box (true),
grid (true: the leg grid as well as the presets), com_max_x (0.15 m).
Prints per row the poses that reach it with the centre of mass x, fewest degrees
from home first; "TIPS" marks the ones past com_max_x.
"""
import itertools
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from transfer_demo import PRE_GRASP, SIDES, TransferDemo  # noqa: E402
from transfer_scene import STATIONS, build_scene  # noqa: E402

# leg grid (rad): the whole upper body forward (j1), up/down (j2), and the lean
# over the pallet (j3 beyond upright = j2 - j1). Kept >= 2 deg inside the limits
# (0..0.937, 0..2.585, 0..2.326): a joint parked on its limit sticks in Fortress.
J1 = [0.05, 0.3, 0.6, 0.9]
J2 = [0.9, 1.2, 1.5, 1.8, 2.1, 2.3, 2.5]
LEAN = [0.0, 0.3, 0.6, 0.9]
J3_RANGE = (0.04, 2.28)


class ReachSweep(TransferDemo):
    def __init__(self):
        super().__init__()
        self.layers = int(self.declare_parameter("layers", 0).value) or self.scene["layers"]
        rows = str(self.declare_parameter("rows", "").value)
        self.rows = [int(r) for r in rows.split(",")] if rows else None
        self.do_box = bool(self.declare_parameter("box", True).value)
        self.grid = bool(self.declare_parameter("grid", True).value)
        self.com_max_x = float(self.declare_parameter("com_max_x", 0.15).value)
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.urdf = None
        self.create_subscription(
            String, "/robot_description", lambda m: setattr(self, "urdf", m.data),
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def load_masses(self):
        """[(link, mass, com offset in the link)] from /robot_description."""
        while self.urdf is None:
            self.get_logger().info("  ... waiting for /robot_description")
            time.sleep(1.0)
        self.masses = []
        for link in ET.fromstring(self.urdf).findall("link"):
            m, o = link.find("inertial/mass"), link.find("inertial/origin")
            if m is not None and float(m.get("value")) > 0:
                xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
                self.masses.append((link.get("name"), float(m.get("value")), xyz))
        self.get_logger().info(f"{len(self.masses)} links, "
                               f"{sum(m for _, m, _ in self.masses):.1f} kg")

    def com_x(self, torso, arms, load):
        """x of the whole robot's centre of mass in base_link, torso at `torso`
        (degrees), arms at `arms`, `load` kg hanging in each hand."""
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [n for n, _, _ in self.masses] + [self.tcp[s] for s in SIDES]
        req.robot_state = RobotState(is_diff=True)
        req.robot_state.joint_state.name = self.torso + [j for s in SIDES for j in self.arm_joints[s]]
        req.robot_state.joint_state.position = [math.radians(d) for d in torso] + [
            v for s in SIDES for v in arms[s]]
        res = self._wait(self.fk.call_async(req), 10.0, "compute_fk")
        poses = dict(zip(res.fk_link_names, res.pose_stamped))
        mx = m_tot = 0.0
        for name, m, (cx, cy, cz) in self.masses:
            p, q = poses[name].pose.position, poses[name].pose.orientation
            # x row of the link's rotation matrix, applied to its com offset
            r0 = (1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z),
                  2 * (q.x * q.z + q.w * q.y))
            mx += m * (p.x + r0[0] * cx + r0[1] * cy + r0[2] * cz)
            m_tot += m
        for s in SIDES:           # the cans hang straight below the (top-down) tcp
            mx += load * poses[self.tcp[s]].pose.position.x
            m_tot += load
        return mx / m_tot

    def poses(self):
        """{label: torso degrees}: the presets, then the grid."""
        out = {n: p for n, p in self.presets.items()}
        if self.grid:
            for j1, j2, d in itertools.product(J1, J2, LEAN):
                j3 = j2 - j1 + d
                if J3_RANGE[0] <= j3 <= J3_RANGE[1]:
                    out[f"({j1:.2f},{j2:.2f},{j3:.2f})"] = [
                        round(math.degrees(v), 1) for v in (j1, j2, j3)] + [0, 0]
        return out

    def load(self, scene, shift_y=0.0, only=None):
        """Replace the MoveIt world with `scene`'s boxes (those named in `only`, if
        given) and pick candidates, moved by shift_y."""
        for n in list(self.objects):
            self.remove(n)
        self.scene = scene
        for b in scene["collision_boxes"]:
            if only is None or b["name"] in only:
                c = b["center"]
                self.objects[b["name"]] = ("box", [c[0], c[1] + shift_y, c[2]], b["size"])
        if only is None:
            for t in scene["targets"]:
                self.objects[t["name"]] = ("can", t["pos"], True)
        self.refresh_scene()

    def reaches(self, bottoms, torso, load, above=PRE_GRASP):
        """The demo's reach test: both hands above and at the handles. Returns
        the centre of mass x at the grasp (see com_x), or None."""
        for dz in (above, 0.0):
            poses = {s: tuple(v + (dz if k == 2 else 0) for k, v in
                              enumerate(self.grasp_pose(bottoms[s]))) for s in SIDES}
            found = self.dual_ik(poses, torso)
            if found is None:
                return None
        return self.com_x(torso, found[0], load)

    def check(self, bottoms, poses, load, above=PRE_GRASP):
        """{pose label: com x} for the poses that reach `bottoms`."""
        out = {}
        for n, p in poses.items():
            c = self.reaches(bottoms, p, load, above)
            if c is not None:
                out[n] = c
        return out

    def report(self, what, found, poses):
        home = poses["home"]
        order = sorted(found, key=lambda n: sum(abs(a - b) for a, b in zip(poses[n], home)))
        ok = [n for n in order if found[n] <= self.com_max_x]
        self.get_logger().info(
            f"{what}: {len(found)} pose(s) reach, {len(ok)} without tipping" + "".join(
                f"\n    {n:22s} {poses[n][:3]}  com x {found[n]:+.3f}"
                f"{'' if found[n] <= self.com_max_x else '  TIPS'}" for n in order))
        return ok

    def sweep(self):
        self.wait_for_servers()
        while not self.ik.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("  ... waiting for /compute_ik")
        while self.odom is None:
            time.sleep(0.5)
        self.fk.wait_for_service()
        self.load_masses()
        # the solver would otherwise try 4x per yaw per arm: 1 is enough here
        solve = self.solve_ik
        self.solve_ik = lambda *a, **k: solve(*a, **dict(k, attempts=1))
        sc, r = self.scene, self.robot["scene"]
        poses = self.poses()
        self.get_logger().info(f"{len(poses)} torso poses, pallet A {self.layers} layers")
        rows = self.rows if self.rows is not None else range(sc["rows"])
        summary = {}
        for row in rows:
            s = build_scene(row, self.layers, r["base_front"], r["base_z"], sc["world_name"])
            self.load(s)
            by_side = {sd: min((t for t in s["targets"] if t["side"] == sd),
                               key=lambda t: abs(t["pos"][1])) for sd in SIDES}
            bottoms = {sd: by_side[sd]["pos"] for sd in SIDES}
            hx = self.grasp_pose(bottoms["left"])
            t0 = time.monotonic()
            found = self.check(bottoms, poses, sc["empty_mass"])
            summary[row + 1] = self.report(
                f"row {row + 1} (handles {hx[0]:.2f} m out, {hx[2]:.2f} m up in "
                f"base_link; {time.monotonic() - t0:.0f} s)", found, poses)
        if self.do_box:
            # the box at pallet B, brought to the robot's current station
            s = build_scene(0, self.layers, r["base_front"], r["base_z"], sc["world_name"],
                            box_on_floor=r.get("box_on_floor", False))
            self.load(s, shift_y=STATIONS["A"] - STATIONS["B"],
                      only={b["name"] for b in s["collision_boxes"] if b["name"].startswith(("box_b", "pallet_b"))})
            slots = {sd: [v[0], v[1] + STATIONS["A"] - STATIONS["B"], v[2] + 0.003]
                     for sd, v in s["b_slots"].items()}
            above = self.robot.get("box_approach", PRE_GRASP)
            found = self.check(slots, poses, sc["filled_mass"], above)
            summary["box"] = self.report(
                f"pallet-B box ({'floor' if s['box_on_floor'] else 'deck'}, loaded)",
                found, poses)
        self.get_logger().info("summary: " + ", ".join(
            f"{'row ' if k != 'box' else ''}{k}: {len(v)}" for k, v in summary.items())
            + " (poses that reach without tipping)")


def main():
    rclpy.init()
    node = ReachSweep()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.sweep()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
