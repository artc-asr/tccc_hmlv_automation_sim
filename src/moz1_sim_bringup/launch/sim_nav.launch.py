"""
sim_nav.launch.py  --  Nav2 + SLAM against the Moz1 Isaac Sim.

Reuses the real robot's TUNED config verbatim (nav2_params.yaml,
slam_toolbox.yaml, pointcloud_to_scan.yaml) but drives every node on
use_sim_time:=true so timestamps match Isaac's clock. The real
nav2_navigation.launch.py hardcodes use_sim_time:=False, so Nav2 is brought up
here via the canonical nav2_bringup launcher pointed at the same params file.

Pipeline (all consuming Isaac's sim topics):
  /livox/lidar (PointCloud2)
    -> pointcloud_to_laserscan -> /scan
    -> slam_toolbox            -> map -> odom TF
  odom -> base_link TF comes from Isaac (attach_sensors.py, ground truth)
  Nav2 (MPPI Omni) -> /cmd_vel -> Isaac mecanum controller (attach_base_control)

Prereq: run sim_bridge.launch.py too (static base_link->livox_frame TF).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare("moz1_navigation_bringup")
    nav2_bringup = FindPackageShare("nav2_bringup")

    pc_to_scan_yaml = PathJoinSubstitution(
        [nav_share, "config", "pointcloud_to_scan.yaml"])
    slam_yaml = PathJoinSubstitution([nav_share, "config", "slam_toolbox.yaml"])
    nav2_yaml = PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"])

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "slam", default_value="true",
            description="Run slam_toolbox (map->odom). Set false if localizing "
                        "against a prebuilt map instead."),

        SetParameter(name="use_sim_time", value=use_sim_time),

        # 3D cloud -> 2D scan (reuses the real tuned yaml + RELIABLE /scan QoS).
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
            parameters=[pc_to_scan_yaml, {"use_sim_time": use_sim_time}],
            output="screen",
        ),

        # slam_toolbox: map -> odom (real launch already threads use_sim_time).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [nav_share, "launch", "slam_mapping.launch.py"])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params": slam_yaml,
            }.items(),
        ),

        # Nav2 servers via the canonical launcher + the real tuned params.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [nav2_bringup, "launch", "navigation_launch.py"])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": LaunchConfiguration("autostart"),
                "params_file": nav2_yaml,
                "use_composition": "False",
            }.items(),
        ),
    ])
