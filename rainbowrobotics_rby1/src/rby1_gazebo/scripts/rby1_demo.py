#!/usr/bin/env python3
"""Simple RB-Y1 demo: move to the rby1-sdk "ready" pose, look around, open/close grippers, drive a bit."""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

deg = math.radians

JOINTS = {
    'torso': [f'torso_{i}' for i in range(6)],
    'right_arm': [f'right_arm_{i}' for i in range(7)],
    'left_arm': [f'left_arm_{i}' for i in range(7)],
    'head': ['head_0', 'head_1'],
    'right_gripper': ['gripper_finger_r1', 'gripper_finger_r2'],
    'left_gripper': ['gripper_finger_l1', 'gripper_finger_l2'],
}

# "Ready" pose used in the rby1-sdk examples.
READY = {
    'torso': [0.0, deg(45), deg(-90), deg(45), 0.0, 0.0],
    'right_arm': [0.0, deg(-5), 0.0, deg(-120), 0.0, deg(70), 0.0],
    'left_arm': [0.0, deg(5), 0.0, deg(-120), 0.0, deg(70), 0.0],
    'head': [0.0, deg(20)],
}
ZERO = {k: [0.0] * len(v) for k, v in JOINTS.items()}
GRIPPER_OPEN = {'right_gripper': [-0.05, 0.05], 'left_gripper': [-0.05, 0.05]}
GRIPPER_CLOSED = {'right_gripper': [0.0, 0.0], 'left_gripper': [0.0, 0.0]}


class Rby1Demo(Node):
    def __init__(self):
        super().__init__('rby1_demo')
        self.traj_clients = {
            g: ActionClient(self, FollowJointTrajectory, f'/{g}_controller/follow_joint_trajectory')
            for g in JOINTS
        }
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, targets: dict, seconds: float):
        """Send goals to several controllers at once and wait for all of them."""
        futures = []
        for group, positions in targets.items():
            client = self.traj_clients[group]
            if not client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'{group}_controller action server not available')
                continue
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS[group]
            point = JointTrajectoryPoint(positions=positions)
            point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [point]
            futures.append((group, client.send_goal_async(goal)))

        for group, fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f'{group}: goal rejected')
                continue
            res = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res, timeout_sec=seconds + 10.0)
            if res.result() is None:
                self.get_logger().error(f'{group}: no result (is the simulation still running?)')
            elif res.result().status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'{group}: finished with status {res.result().status}')

    def drive(self, vx: float, wz: float, seconds: float):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        end = time.time() + seconds
        while time.time() < end:
            self.cmd_vel.publish(msg)
            time.sleep(0.05)
        self.cmd_vel.publish(Twist())


def main():
    rclpy.init()
    node = Rby1Demo()
    log = node.get_logger()

    log.info('Moving to ready pose')
    node.move(READY, 4.0)
    log.info('Opening grippers')
    node.move(GRIPPER_OPEN, 1.5)
    log.info('Looking left / right')
    node.move({'head': [deg(40), deg(20)]}, 1.5)
    node.move({'head': [deg(-40), deg(20)]}, 2.0)
    node.move({'head': READY['head']}, 1.5)
    log.info('Closing grippers')
    node.move(GRIPPER_CLOSED, 1.5)
    log.info('Driving back and forth, then turning (ends near the start pose)')
    node.drive(-0.2, 0.0, 2.0)
    node.drive(0.2, 0.0, 2.0)
    node.drive(0.0, 0.5, 2.0)
    node.drive(0.0, -0.5, 2.0)
    log.info('Back to zero pose')
    node.move(ZERO, 4.0)
    log.info('Demo done')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
