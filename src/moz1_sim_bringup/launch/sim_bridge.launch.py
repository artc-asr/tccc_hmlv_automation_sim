"""
sim_bridge.launch.py  --  host-side glue between Isaac Sim and the real stack.

Starts the pieces that adapt Isaac's raw topics to the canonical contract:
  * joint_name_remap : /isaac/joint_states (sim names) <-> /joint_states,
                       /joint_command <-> /isaac/joint_command (canonical).
  * static TF        : base_link -> livox_frame (mirrors livox_bringup, z=0.4),
                       since the sim does not run the real Livox driver.
  * pc2_to_livox     : OPTIONAL PointCloud2 -> CustomMsg for the FAST-LIO2 path
                       (enable with fastlio:=true).

Everything runs on use_sim_time so timestamps match Isaac's sim clock.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_share = FindPackageShare("moz1_sim_bridge")
    default_map = PathJoinSubstitution([bridge_share, "config", "joint_name_map.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "joint_map", default_value=default_map,
            description="YAML mapping sim joint names -> canonical URDF names."),
        DeclareLaunchArgument("tf_z", default_value="0.4",
                              description="base_link -> livox_frame Z offset (m)."),
        DeclareLaunchArgument("fastlio", default_value="false",
                              description="Also run pc2_to_livox for the FAST-LIO2 path."),

        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),

        # Canonicalise joint names both directions.
        Node(
            package="moz1_sim_bridge",
            executable="joint_name_remap",
            name="joint_name_remap",
            output="screen",
            parameters=[{
                "map_file": LaunchConfiguration("joint_map"),
                "in_state_topic": "/isaac/joint_states",
                "out_state_topic": "/joint_states",
                "in_command_topic": "/joint_command",
                "out_command_topic": "/isaac/joint_command",
            }],
        ),

        # base_link -> livox_frame (sim has no real Livox driver to publish it).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_livox_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", LaunchConfiguration("tf_z"),
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "livox_frame",
            ],
        ),

        # Optional: feed FAST-LIO2 (lidar_type=1 / CustomMsg).
        Node(
            package="moz1_sim_bridge",
            executable="pc2_to_livox",
            name="pc2_to_livox",
            output="screen",
            condition=IfCondition(LaunchConfiguration("fastlio")),
            parameters=[{
                "in_topic": "/livox/lidar",
                "out_topic": "/livox/lidar_custom",
                "scan_rate_hz": 10.0,
            }],
        ),
    ])
