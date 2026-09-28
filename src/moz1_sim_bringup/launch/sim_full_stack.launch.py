"""
sim_full_stack.launch.py  --  full host-side stack for the Moz1 Isaac Sim.

One command to drive the simulation with the unchanged real-robot stack:
  * sim_bridge : joint_name_remap + static base_link->livox_frame TF (+opt FAST-LIO2)
  * sim_nav    : pointcloud_to_laserscan + slam_toolbox + Nav2 (MPPI Omni)
  * sim_moveit : MoveIt move_group + topic_based_ros2_control + RViz

Prerequisites (Isaac Sim side, pasted into the Script Editor with sim PLAYING):
  1. attach_sensors.py       -> /livox/lidar, /livox/imu, /odom, /tf
  2. attach_base_control.py  -> SUB /cmd_vel (mecanum)
  3. attach_joint_io.py      -> /isaac/joint_states, SUB /isaac/joint_command

Both sides MUST share ROS_DOMAIN_ID=33 and reach each other over DDS.

Toggle halves with nav:=false / moveit:=false. Enable the FAST-LIO2 path with
fastlio:=true (also point the FAST-LIO2 config at /livox/lidar_custom).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("moz1_sim_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    def _inc(name, extra=None, condition=None):
        args = {"use_sim_time": use_sim_time}
        if extra:
            args.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([share, "launch", name])),
            launch_arguments=args.items(),
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("fastlio", default_value="false"),

        _inc("sim_bridge.launch.py",
             extra={"fastlio": LaunchConfiguration("fastlio")}),
        _inc("sim_nav.launch.py",
             condition=IfCondition(LaunchConfiguration("nav"))),
        # sim_moveit hardcodes use_sim_time internally; no extra args needed.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([share, "launch", "sim_moveit.launch.py"])),
            condition=IfCondition(LaunchConfiguration("moveit")),
        ),
    ])
