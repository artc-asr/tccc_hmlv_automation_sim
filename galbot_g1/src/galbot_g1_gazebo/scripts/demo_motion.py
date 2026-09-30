#!/usr/bin/env python3
"""Exercise every Galbot G1 controller in simulation: leg, head, arms, grippers and base."""

import math

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

LEG = [f'leg_joint{i}' for i in range(1, 6)]
HEAD = ['head_joint1', 'head_joint2']
LEFT_ARM = [f'left_arm_joint{i}' for i in range(1, 8)]
RIGHT_ARM = [f'right_arm_joint{i}' for i in range(1, 8)]

# Arm "ready" pose (grippers in front of the torso); the right arm mirrors the left.
LEFT_READY = [1.45, -0.85, 0.0, -2.07, -1.58, -0.72, 0.0]
LEFT_WAVE = [1.45, -1.3, 0.0, -1.2, -1.58, 0.0, 0.0]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.2


def leg_pose(hip, knee):
    """Leg pose that keeps the torso upright (leg_joint3 = leg_joint2 - leg_joint1)."""
    return [hip, knee, knee - hip, 0.0, 0.0]


def mirror(q):
    return [-v for v in q]


class Demo(Node):

    def __init__(self):
        super().__init__('galbot_g1_demo')
        self.trajectory_clients = {
            name: ActionClient(self, FollowJointTrajectory,
                               f'/{name}_controller/follow_joint_trajectory')
            for name in ('leg', 'head', 'left_arm', 'right_arm')
        }
        self.gripper_clients = {
            side: ActionClient(self, GripperCommand, f'/{side}_gripper_controller/gripper_cmd')
            for side in ('left', 'right')
        }
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def wait_for_servers(self):
        for client in [*self.trajectory_clients.values(), *self.gripper_clients.values()]:
            self.get_logger().info(f'Waiting for {client._action_name} ...')
            client.wait_for_server()

    def move(self, goals, seconds):
        """Send joint goals {controller: (joints, positions)} in parallel and wait for all."""
        futures = []
        for controller, (joints, positions) in goals.items():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joints
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=positions,
                time_from_start=Duration(seconds=seconds).to_msg())]
            futures.append(self.trajectory_clients[controller].send_goal_async(goal))
        self._wait_for_results(futures)

    def grip(self, position):
        futures = []
        for client in self.gripper_clients.values():
            goal = GripperCommand.Goal()
            goal.command.position = position
            goal.command.max_effort = 50.0
            futures.append(client.send_goal_async(goal))
        self._wait_for_results(futures)

    def _wait_for_results(self, goal_futures):
        result_futures = []
        for future in goal_futures:
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error('Goal rejected')
                continue
            result_futures.append(handle.get_result_async())
        for future in result_futures:
            rclpy.spin_until_future_complete(self, future)
            result = future.result().result
            if getattr(result, 'error_code', 0) != 0:
                self.get_logger().error(
                    f'Trajectory failed ({result.error_code}): {result.error_string}')

    def drive(self, vx, vy, wz, seconds):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        end = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < end:
            self.cmd_vel.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_vel.publish(Twist())

    def run(self):
        self.wait_for_servers()
        log = self.get_logger().info

        log('Ready pose')
        self.move({'leg': (LEG, leg_pose(0.6, 1.8)),
                   'head': (HEAD, [0.0, 0.0]),
                   'left_arm': (LEFT_ARM, LEFT_READY),
                   'right_arm': (RIGHT_ARM, mirror(LEFT_READY))}, 3.0)

        log('Look around')
        self.move({'head': (HEAD, [0.8, 0.3])}, 1.5)
        self.move({'head': (HEAD, [-0.8, 0.3])}, 2.5)
        self.move({'head': (HEAD, [0.0, 0.35])}, 1.5)

        log('Torso down, then up')
        self.move({'leg': (LEG, leg_pose(0.3, 1.3))}, 3.0)
        self.move({'leg': (LEG, leg_pose(0.85, 2.3))}, 4.0)
        self.move({'leg': (LEG, leg_pose(0.6, 1.8))}, 3.0)

        log('Wave both arms')
        for _ in range(2):
            self.move({'left_arm': (LEFT_ARM, LEFT_WAVE),
                       'right_arm': (RIGHT_ARM, mirror(LEFT_WAVE))}, 1.5)
            self.move({'left_arm': (LEFT_ARM, LEFT_READY),
                       'right_arm': (RIGHT_ARM, mirror(LEFT_READY))}, 1.5)

        log('Grippers close / open')
        self.grip(GRIPPER_CLOSED)
        self.grip(GRIPPER_OPEN)

        log('Base: back, strafe, rotate')
        self.drive(-0.2, 0.0, 0.0, 2.0)
        self.drive(0.0, 0.2, 0.0, 2.0)
        self.drive(0.0, 0.0, math.pi / 4, 2.0)
        self.drive(0.0, 0.0, -math.pi / 4, 2.0)
        self.drive(0.0, -0.2, 0.0, 2.0)
        self.drive(0.2, 0.0, 0.0, 2.0)
        log('Done')


def main():
    rclpy.init()
    node = Demo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():      # Ctrl-C has already shut the context down
            node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
