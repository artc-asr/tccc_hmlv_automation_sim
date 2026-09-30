#!/usr/bin/env python3
"""duo_demo.py — one cycle of the two-robot line (launch/duo.launch.py).

    duo_demo.py g1 | moz1 | supervisor      (one node each; the robots in /g1, /moz1)

The cell (scripts/duo_scene.py) starts as a running line: two FILLED cans at the
belt's unload end, two EMPTY cans under the filling station in the middle.

  g1          at pallet A: picks an empty pair (transfer_demo's pallet-A pick: 4
              layers, row 4, least torso change), drives to the belt's LOAD station
              and sets it down -> "placed"; then home and back to pallet A -> "home"
  moz1        at the UNLOAD station: picks the filled pair, curls in, drives to
              pallet B and places it in the box on the deck -> "placed"; then home
              and back to the unload station -> "home"
  supervisor  once both have placed: the belt fills the pair under the filler and
              indexes one station (IndexingConveyor) — filled pair to UNLOAD, the
              G1's pair to the filler — while the robots drive back; when both are
              home, checks the line against Gazebo: the start again.

The robots run concurrently; each is a TransferDemo (the one-robot demo's steps) in
its own namespace. Events: /<ns>/duo_event (latched String: placed, home, failed).
Phases for the recording: /<ns>/transfer_demo/phase per robot, /duo/phase for the
line (cycle, conveyor, done | failed).
"""
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Empty, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from transfer_demo import SIDES, StepFailed, TransferDemo  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
ROBOTS = ("g1", "moz1")


class DuoRobot(TransferDemo):
    def __init__(self, role):
        super().__init__()
        self.role = role
        self.event_pub = self.create_publisher(String, "duo_event", LATCHED)
        # the cell is laid out from the G1's chassis front; this robot's stands this
        # much further back (the Moz1's: 14 mm), so it stops that much further back
        self.x_off = self.scene["base_front"] - self.robot["scene"]["base_front"]

    def drive_to(self, x, y, name, **kw):
        super().drive_to(x + self.x_off, y, name, **kw)

    def event(self, name):
        self.event_pub.publish(String(data=name))
        self.get_logger().info(f"event: {name}")      # the log is per robot already

    def setup_scene(self):
        """The cell, plus the four cans on the belt: the pair this robot picks as
        bodies only (the fingers straddle the handles), the others full height."""
        super().setup_scene()
        for c in self.scene["belt_cans"]:
            mine = self.role == "moz1" and c["station"] == "unload"
            self.objects[c["name"]] = ("can", c["pos"], mine)
        self.refresh_scene()

    def run(self):
        self.startup()
        self.setup_scene()
        s = self.scene
        if self.role == "g1":
            self.phase("pick_a")
            names, quats = self.pick_from_pallet_a()
            self.place_on_belt(names, quats, "load")
            self.event("placed")
            back = "A"
        else:
            cans = {c["side"]: c for c in s["belt_cans"] if c["station"] == "unload"}
            names = {sd: cans[sd]["name"] for sd in SIDES}
            self.dual_gripper(self.g_open)
            quats = self.pick_from_belt(names, {sd: cans[sd]["pos"] for sd in SIDES}, "unload")
            self.place_in_box(names, quats)
            self.event("placed")
            back = "unload"
        self.phase("home")
        self.go_home()
        self.drive_to(*s["stations"][back], f"start ({back})")
        self.phase("done")
        self.event("home")


class Supervisor(Node):
    def __init__(self):
        super().__init__("duo_supervisor")
        import yaml
        with open(self.declare_parameter("scene_file", "").value) as f:
            self.scene = yaml.safe_load(f)
        self.events = {r: set() for r in ROBOTS}     # every event each robot reached
        self.changed = threading.Event()
        for r in ROBOTS:
            self.create_subscription(String, f"/{r}/duo_event",
                                     lambda m, r=r: self._on_event(r, m.data), LATCHED)
        self.phase_pub = self.create_publisher(String, "/duo/phase", LATCHED)
        self.conveyor_start = self.create_publisher(Empty, "/conveyor/start", 10)
        # the filled pair's new colour for RViz and the recording (the belt recolours
        # it in Gazebo itself; the web replay only sees these)
        self.recolor_pub = self.create_publisher(String, "/scene_markers/recolor", 10)
        self.conveyor_done = threading.Event()
        self.create_subscription(Empty, "/conveyor/done", lambda _m: self.conveyor_done.set(), 10)

    def _on_event(self, robot, name):
        # a set, not the latest: the G1 is usually "home" again before the Moz1 has
        # even "placed" — waiting for both to be "placed" at once never ended
        self.events[robot].add(name)
        self.changed.set()

    def wait_all(self, name, timeout):
        t0 = time.monotonic()
        while True:
            if any("failed" in e for e in self.events.values()):
                raise StepFailed(f"a robot failed: {self.events}")
            if all(name in e for e in self.events.values()):
                return
            left = timeout - (time.monotonic() - t0)
            if left <= 0:
                raise StepFailed(f"timed out waiting for both robots to be '{name}': "
                                 f"{self.events}")
            self.changed.wait(min(left, 5.0))
            self.changed.clear()

    def gz_positions(self):
        topic = f"/world/{self.scene['world_name']}/pose/info"
        out = subprocess.run(["ign", "topic", "-e", "-t", topic, "-n", "1"],
                             capture_output=True, text=True, timeout=15).stdout
        found = {}
        for block in re.findall(r"pose \{(.*?)\n\}", out, re.S):
            name = re.search(r'name: "([^"]+)"', block)
            pos = re.search(r"position \{(.*?)\}", block, re.S)
            if name and pos:
                v = {k: float(x) for k, x in re.findall(r"(\w): ([-\d.e]+)", pos.group(1))}
                found[name.group(1)] = (v.get("x", 0.0), v.get("y", 0.0), v.get("z", 0.0))
        return found

    def check_line(self):
        """Both pairs where a running line starts: the formerly empty pair (filled
        now) at the unload station, the G1's pair under the filler."""
        s, now = self.scene, self.gz_positions()
        st, step, top = s["stations"], s["step"], s["conveyor"]["top"]
        report = []
        for c in s["belt_cans"]:
            if c["station"] != "fill":
                continue
            want = (c["pos"][0], c["pos"][1] + step, top)
            got = now.get(c["name"])
            if got is None or math.dist(got, want) > 0.04:
                raise StepFailed(f"{c['name']} is not at the unload station (Gazebo: {got})")
            report.append(f"{c['name']} (filled) at unload")
        fill_y = st["fill"][1]
        at_fill = [n for n, p in now.items() if n.startswith("jerrycan_a")
                   and abs(p[1] - fill_y) < 0.3 and abs(p[2] - top) < 0.03]
        if len(at_fill) != 2:
            raise StepFailed(f"expected the G1's pair under the filler, found {at_fill}")
        report.append(f"{' + '.join(sorted(at_fill))} (empty) under the filler")
        return report

    def run(self):
        self.get_logger().info("duo line: G1 pallet A -> belt, Moz1 belt -> pallet B, "
                               "concurrently; then the belt fills and indexes")
        self.phase_pub.publish(String(data="cycle"))
        self.wait_all("placed", 900.0)
        self.get_logger().info("both pairs placed: belt fills the middle pair and indexes")
        self.phase_pub.publish(String(data="conveyor"))
        self.conveyor_done.clear()
        for c in self.scene["belt_cans"]:           # the pair under the filler: filled now
            if c["station"] == "fill":
                self.recolor_pub.publish(String(data=f"{c['name']} {self.scene['filled_rgba']}"))
        for _ in range(3):
            self.conveyor_start.publish(Empty())
            if self.conveyor_done.wait(timeout=20.0):
                break
        else:
            raise StepFailed("the belt did not report done")
        self.get_logger().info("belt indexed; waiting for both robots to be back")
        self.wait_all("home", 300.0)
        for line in self.check_line():
            self.get_logger().info(f"  {line}")
        self.phase_pub.publish(String(data="done"))
        self.get_logger().info("done: one cycle — 2 filled cans in the box on pallet B, "
                               "the line back at its start")


def main():
    role = (remove_ros_args(sys.argv)[1:] or ["supervisor"])[0]
    rclpy.init()
    node = Supervisor() if role == "supervisor" else DuoRobot(role)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except StepFailed as exc:
        node.get_logger().error(f"stopped: {exc}")
        if role == "supervisor":
            node.phase_pub.publish(String(data="failed"))
        else:
            node.phase("failed")
            node.event("failed")
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if role != "supervisor":
            node.cmd_vel.publish(Twist())
        executor.shutdown()
        spin.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
