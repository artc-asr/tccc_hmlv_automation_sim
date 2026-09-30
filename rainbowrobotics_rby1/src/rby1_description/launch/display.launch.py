"""Visualize the RB-Y1 URDF in RViz with joint sliders (no simulation)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rby1_description')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'rby1.urdf.xacro'])]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show joint slider GUI'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'rby1.rviz'])]),
    ])
