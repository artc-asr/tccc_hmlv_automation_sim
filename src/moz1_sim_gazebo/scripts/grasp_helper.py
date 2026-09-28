#!/usr/bin/env python3
"""grasp_helper.py — emulate a physical grasp in Gazebo.

Closing a simple gripper on an object in gz won't lift it (friction alone slips),
so this node welds the object to the gripper via the gz **DetachableJoint** system
(configured in the robot xacro) when the gripper CLOSES, and releases it when the
gripper OPENS. The weld is created at the current relative pose, so the object then
moves rigidly with the hand — a believable pick → carry → place.

It watches `/joint_states` for `left_gripper_joint` / `right_gripper_joint` and, on a
close edge, publishes std_msgs/Empty on `/grasp/<side>/attach` (bridged to the gz
DetachableJoint attach topic); on an open edge it publishes `/grasp/<side>/detach`.

Why a sim-side node (not the manipulation skill): on the REAL robot, closing the
gripper physically grasps — the skill just commands the gripper. This node makes the
SIM behave the same way, so the manipulation skill / orchestration stay entirely
sim/real-agnostic (the digital-twin principle). Left gripper ↔ bearing_ring, right
gripper ↔ sample_bottle (the DetachableJoint child_model mapping in the xacro).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

# Gripper joint position (m): open ≈ 0.03, closed grasp ≈ 0.005 (manipulation_targets).
CLOSED_THRESH = 0.015   # below this -> treat as closed (grasp)
OPEN_THRESH = 0.022     # above this -> treat as open (release); hysteresis gap avoids chatter


class GraspHelper(Node):
    def __init__(self):
        super().__init__("grasp_helper")
        self.pub = {}
        for side in ("left", "right"):
            self.pub[(side, "attach")] = self.create_publisher(Empty, f"/grasp/{side}/attach", 10)
            self.pub[(side, "detach")] = self.create_publisher(Empty, f"/grasp/{side}/detach", 10)
        self.closed = {"left": False, "right": False}
        self._did_startup = False
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        # Ensure a deterministic DETACHED initial state (in case the DetachableJoint
        # welds at spawn): publish detach once after the bridge is up.
        self.create_timer(2.0, self._startup_detach)
        self.get_logger().info(
            "grasp_helper: attach on gripper close, detach on open "
            "(left↔bearing_ring, right↔sample_bottle)")

    def _startup_detach(self):
        if self._did_startup:
            return
        for side in ("left", "right"):
            self.pub[(side, "detach")].publish(Empty())
        self._did_startup = True

    def _on_js(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        for side in ("left", "right"):
            p = pos.get(f"{side}_gripper_joint")
            if p is None:
                continue
            if not self.closed[side] and p < CLOSED_THRESH:
                self.closed[side] = True
                self.pub[(side, "attach")].publish(Empty())
                self.get_logger().info(f"{side} gripper closed → attach")
            elif self.closed[side] and p > OPEN_THRESH:
                self.closed[side] = False
                self.pub[(side, "detach")].publish(Empty())
                self.get_logger().info(f"{side} gripper opened → detach")


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
