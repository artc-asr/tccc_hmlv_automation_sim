#!/usr/bin/env python3
"""record_run.py — record a transfer demo run for the web replay (docs/).

    ros2 launch hmlv_cell_gazebo transfer.launch.py rviz:=false record:=/tmp/run_moz1

Samples at `rate` Hz of sim time: the robot's joints (/joint_states), the base
pose and every jerry can's pose (/tf: <any> -> base_link, gz_world -> jerrycan_*,
as gz_pose_tf broadcasts them). Also keeps, with their sim times, the demo's steps
(/transfer_demo/phase), its log lines (/rosout, node transfer_demo) and the can
recolours (/scene_markers/recolor). Stops `tail` s after the demo publishes "done"
or "failed" (or after max_duration s, or on Ctrl-C) and writes to `out_dir`:

    recording.json   the samples and events
    robot.urdf       /robot_description as the sim used it
    world.sdf        a copy of world_file (the static scene)

tools/web_replay/build.py turns that into the page's data.
"""
import json
import os
import shutil
import signal

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ERROR = 40                  # rcl_interfaces Log.ERROR (a bytes constant in Humble)
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _pose(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return [round(v, 4) for v in (t.x, t.y, t.z)] + [round(v, 5) for v in (q.x, q.y, q.z, q.w)]


class RunRecorder(Node):
    def __init__(self):
        super().__init__("record_run")
        self.out_dir = self.declare_parameter("out_dir", "/tmp/hmlv_run").value
        self.world_file = self.declare_parameter("world_file", "").value
        self.robot = self.declare_parameter("robot", "").value
        rate = self.declare_parameter("rate", 20.0).value
        self.tail = self.declare_parameter("tail", 3.0).value
        self.max_duration = self.declare_parameter("max_duration", 1200.0).value
        self.can_prefix = self.declare_parameter("can_prefix", "jerrycan_").value

        self.joints = {}            # name -> position (latest)
        self.base = None            # latest base_link pose
        self.cans = {}              # name -> latest pose
        self.urdf = None
        self.t0 = None
        self.end_at = None
        self.written = False
        self.rec = {"rate": rate, "joint_names": [], "t": [], "base": [], "joints": [],
                    "cans": {}, "phases": [], "logs": [], "recolors": []}

        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        self.create_subscription(String, "/robot_description", self._on_urdf, LATCHED)
        self.create_subscription(String, "/transfer_demo/phase", self._on_phase, LATCHED)
        self.create_subscription(String, "/scene_markers/recolor", self._on_recolor, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_timer(1.0 / rate, self._sample)
        self.get_logger().info(f"recording at {rate:.0f} Hz into {self.out_dir}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _rel(self, t):
        return round(t - self.t0, 3) if self.t0 is not None else 0.0

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def _on_tf(self, msg):
        for tf in msg.transforms:
            child = tf.child_frame_id
            if child == "base_link":
                self.base = _pose(tf)
            elif child.startswith(self.can_prefix):
                self.cans[child] = _pose(tf)

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_phase(self, msg):
        self.rec["phases"].append({"t": self._rel(self._now()), "phase": msg.data})
        self.get_logger().info(f"phase: {msg.data}")
        if msg.data in ("done", "failed") and self.end_at is None:
            self.end_at = self._now() + self.tail

    def _on_recolor(self, msg):
        name, *rgba = msg.data.split()
        self.rec["recolors"].append({"t": self._rel(self._now()), "model": name,
                                     "rgba": [float(v) for v in rgba]})

    def _on_log(self, msg):
        if msg.name != "transfer_demo":
            return
        # at receipt, in sim time: /rosout stamps are wall-clock time
        self.rec["logs"].append({"t": self._rel(self._now()), "level": int(msg.level),
                                 "msg": msg.msg})
        if msg.level >= ERROR and msg.msg.startswith("stopped:") and self.end_at is None:
            self.end_at = self._now() + self.tail

    def _sample(self):
        now = self._now()
        if now <= 0.0 or self.base is None or not self.joints:
            return                                  # no /clock or no robot yet
        if self.t0 is None:
            self.t0 = now
            self.rec["joint_names"] = sorted(self.joints)
            self.get_logger().info(f"first sample: {len(self.joints)} joints, "
                                   f"{len(self.cans)} cans")
        r = self.rec
        r["t"].append(self._rel(now))
        r["base"].append(self.base)
        r["joints"].append([round(self.joints.get(n, 0.0), 4) for n in r["joint_names"]])
        n = len(r["t"])
        for name, pose in self.cans.items():
            # a can first seen late is back-filled with its first pose
            r["cans"].setdefault(name, [pose] * (n - 1)).append(pose)
        if (self.end_at is not None and now >= self.end_at) or \
                now - self.t0 > self.max_duration:
            self.write()
            raise SystemExit

    def write(self):
        if self.written or self.t0 is None:
            return
        self.written = True
        os.makedirs(self.out_dir, exist_ok=True)
        self.rec["robot"] = self.robot
        with open(os.path.join(self.out_dir, "recording.json"), "w") as f:
            json.dump(self.rec, f, separators=(",", ":"))
        if self.urdf:
            with open(os.path.join(self.out_dir, "robot.urdf"), "w") as f:
                f.write(self.urdf)
        else:
            self.get_logger().warn("no /robot_description received: robot.urdf not written")
        if self.world_file:
            shutil.copy(self.world_file, os.path.join(self.out_dir, "world.sdf"))
        self.get_logger().info(
            f"wrote {len(self.rec['t'])} samples ({self.rec['t'][-1]:.0f} s), "
            f"{len(self.rec['phases'])} phases, {len(self.rec['logs'])} log lines "
            f"to {self.out_dir}")


def main():
    rclpy.init()
    node = RunRecorder()
    def on_sigint(*_):              # ros2 launch stops nodes with SIGINT: keep what there is
        node.write()
        raise SystemExit
    signal.signal(signal.SIGINT, on_sigint)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        node.write()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
