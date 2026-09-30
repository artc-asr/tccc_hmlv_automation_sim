# ============================================================================
# display.launch.py  -  Moz1 model visualization in RViz2 (NO hardware).
#
# Starts:
#   * robot_state_publisher       (URDF -> TF)
#   * joint_state_publisher_gui   (sliders, gui:=true)  OR
#     joint_state_publisher       (home pose,  gui:=false)
#   * rviz2                       (RobotModel + TF view)
#
# This is the Milestone-1 acceptance command:
#     ros2 launch moz1_description display.launch.py
#
# SAFETY: this launch only renders a model. It does NOT connect to the robot,
# does NOT load any controllers, and CANNOT cause physical motion.
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("moz1_description")
    xacro_file = os.path.join(pkg, "urdf", "moz1.urdf.xacro")
    rviz_file = os.path.join(pkg, "rviz", "moz1_display.rviz")
    home_file = os.path.join(pkg, "config", "home_joint_states.yaml")

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true", choices=["true", "false"],
            description="true: joint_state_publisher_gui sliders; "
                        "false: static home pose.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"],
            description="Start RViz2.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Interactive sliders (default).
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        # Static home pose when gui:=false.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(gui),
            parameters=[home_file],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_file],
        ),
    ])
