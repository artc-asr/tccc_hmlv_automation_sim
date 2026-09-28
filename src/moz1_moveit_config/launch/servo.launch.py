# ============================================================================
# servo.launch.py  -  MoveIt Servo node for the LEFT arm (PRD §9.5).
#
# >>> VERSION 1 IS CONFIG-ONLY. <<<
# This launches the moveit_servo node so the config can be exercised, but:
#   * It MUST be run alongside demo.launch.py (which provides the planning scene
#     and the MOCK controllers). Servo's output goes to the mock
#     /left_arm_controller, i.e. SIMULATION ONLY.
#   * Nothing here talks to the MozRobot SDK or real hardware. Do NOT point
#     command_out_topic at a real-robot bridge until that bridge is validated.
#
# Usage (two terminals):
#   ros2 launch moz1_moveit_config demo.launch.py
#   ros2 launch moz1_moveit_config servo.launch.py
# Then publish geometry_msgs/TwistStamped to /servo_node/delta_twist_cmds and
# call the /servo_node/start_servo service.
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    servo_yaml = _load_yaml(
        os.path.join(
            get_package_share_directory("moz1_moveit_config"), "config", "servo.yaml"
        )
    )
    # moveit_servo expects its parameters under the "moveit_servo" key.
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([servo_node])
