#!/usr/bin/env python3
"""grasp_helper.py — emulate a physical grasp in Gazebo.

Closing a simple gripper on an object in gz won't lift it (friction alone slips),
so this node has the object held rigidly by the gripper (moz1_sim_gazebo's
KinematicGrasp gz system, src/kinematic_grasp.cc) when the gripper CLOSES, and
released when it OPENS. The hold keeps the current relative pose, so the object
then moves rigidly with the hand — a believable pick → carry → place.

sim_gazebo.launch.py injects one grasp plugin per (side, object) from its
`grasp_targets` argument, each on its own gz topic pair
`/grasp/<side>/<object>/{attach,detach}` (bridged from ROS std_msgs/Empty). This
node picks which one to fire:

  * params `left_targets` / `right_targets` — the objects each gripper may weld
    (set by the launch). The first entry is the initial target.
  * `/grasp/<side>/target` (std_msgs/String) — select the object the NEXT close
    will weld. A pick-and-place script publishes this before it closes.
  * `/joint_states` — on a `<side>_gripper_joint` close edge publish attach for the
    selected target; on an open edge detach whatever that side holds. Params
    `closed_threshold` / `open_threshold` (defaults: the Moz1's stroke) set the
    edges; a closed_threshold ABOVE open_threshold means the joint grows as the
    gripper closes (e.g. the Galbot G1's, 0 open .. 1.703 closed).
  * `/grasp/<side>/{attach,detach}` (std_msgs/Empty) — manual override, same
    effect as the close/open edge.

Why a sim-side node (not the manipulation skill): on the REAL robot, closing the
gripper physically grasps — the skill just commands the gripper. This node makes the
SIM behave the same way, so the manipulation skill / orchestration stay sim/real-
agnostic apart from naming the target (the digital-twin principle).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

# Gripper joint = SDK stroke, 0.0 (closed) .. 0.12 (open); see moz1_gripper_macros.
# Weld BEFORE the jaw reaches the object: 0.04 ≈ 38 mm gap (gripper_cmd_to_gap.csv),
# so e.g. a 24 mm handle bar is welded at its untouched pose.
CLOSED_THRESH = 0.04    # below this -> treat as closed (grasp)
OPEN_THRESH = 0.06      # above this -> treat as open (release); hysteresis gap avoids chatter
SIDES = ("left", "right")


class GraspHelper(Node):
    def __init__(self):
        super().__init__("grasp_helper")
        self.targets = {}
        for side in SIDES:
            names = self.declare_parameter(f"{side}_targets", [""]).value
            self.targets[side] = [n for n in names if n]
        self.pub = {}
        for side in SIDES:
            for model in self.targets[side]:
                for act in ("attach", "detach"):
                    self.pub[(side, model, act)] = self.create_publisher(
                        Empty, f"/grasp/{side}/{model}/{act}", 10)
        self.selected = {s: (self.targets[s][0] if self.targets[s] else None) for s in SIDES}
        self.held = {s: None for s in SIDES}
        self.closed = {s: False for s in SIDES}
        self.closed_at = float(self.declare_parameter("closed_threshold", CLOSED_THRESH).value)
        self.open_at = float(self.declare_parameter("open_threshold", OPEN_THRESH).value)
        # +1: the joint shrinks as the gripper closes (Moz1), -1: it grows (G1)
        self.sign = 1.0 if self.closed_at < self.open_at else -1.0
        self._did_startup = False

        for side in SIDES:
            self.create_subscription(String, f"/grasp/{side}/target",
                                     lambda m, s=side: self._on_target(s, m.data), 10)
            self.create_subscription(Empty, f"/grasp/{side}/attach",
                                     lambda _m, s=side: self._attach(s), 10)
            self.create_subscription(Empty, f"/grasp/{side}/detach",
                                     lambda _m, s=side: self._detach(s), 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        # Ensure a deterministic DETACHED initial state: publish detach for every
        # target once the bridge is up.
        self.create_timer(2.0, self._startup_detach)
        self.get_logger().info(
            "grasp_helper: attach on gripper close, detach on open "
            f"(left↔{self.targets['left']}, right↔{self.targets['right']})")

    def _startup_detach(self):
        if self._did_startup:
            return
        for (side, model, act), pub in self.pub.items():
            if act == "detach":
                pub.publish(Empty())
        self._did_startup = True

    def _on_target(self, side, model):
        if model not in self.targets[side]:
            self.get_logger().error(
                f"{side} target '{model}' has no grasp plugin "
                f"(known: {self.targets[side]}) — add it to grasp_targets")
            return
        self.selected[side] = model
        self.get_logger().info(f"{side} target → {model}")

    def _attach(self, side):
        model = self.selected[side]
        if model is None or self.held[side] is not None:
            return
        self.pub[(side, model, "attach")].publish(Empty())
        self.held[side] = model
        self.get_logger().info(f"{side} gripper closed → attach {model}")

    def _detach(self, side):
        model = self.held[side]
        if model is None:
            return
        self.pub[(side, model, "detach")].publish(Empty())
        self.held[side] = None
        self.get_logger().info(f"{side} gripper opened → detach {model}")

    def _on_js(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        for side in SIDES:
            p = pos.get(f"{side}_gripper_joint")
            if p is None:
                continue
            if not self.closed[side] and self.sign * (p - self.closed_at) < 0:
                self.closed[side] = True
                self._attach(side)
            elif self.closed[side] and self.sign * (p - self.open_at) > 0:
                self.closed[side] = False
                self._detach(side)


def main():
    rclpy.init()
    node = GraspHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
