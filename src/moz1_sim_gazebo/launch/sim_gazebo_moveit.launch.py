"""sim_gazebo_moveit.launch.py  --  MoveIt move_group against the Gazebo Moz1 arm.

Unlike the Isaac path (which needs topic_based_ros2_control + joint_name_remap),
Gazebo runs the REAL gazebo_ros2_control controllers with the canonical joint
names, so move_group drives them directly — no remap, no topic bridge. This
reuses moz1_moveit_config's SRDF/kinematics/planners byte-identical; only the
robot_description carries the Gazebo hardware (from sim_gazebo.launch.py).

Prereq: run sim_gazebo.launch.py (or this includes it) so the controllers exist.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gz_share = FindPackageShare("moz1_sim_gazebo")

    # SRDF, kinematics, joint limits, planners from moz1_moveit_config (unchanged);
    # move_group plans, the Gazebo controllers execute.
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), {
            "use_sim_time": True,
            # MoveIt refuses a trajectory whose start is further than this (rad)
            # from the current state. 0.01 is MoveIt's default; an arm carrying a
            # payload sags past it under gz's position control.
            "trajectory_execution.allowed_start_tolerance":
                LaunchConfiguration("allowed_start_tolerance")}],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[moveit_config.robot_description,            # URDF
                    moveit_config.robot_description_semantic,   # SRDF (was missing)
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines, {"use_sim_time": True}],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        # forwarded to the sim so `... lidar:=false gui:=false` reaches gz.
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("spawn_delay", default_value="8.0"),
        # default fix_base:=true — a pinned, upright base is what you want for arm
        # planning, and it provides the `world` frame the SRDF virtual_joint needs.
        DeclareLaunchArgument("fix_base", default_value="true"),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [gz_share, "worlds", "moz1_pickplace.world"])),
        DeclareLaunchArgument("grasp_targets",
                              default_value="left:bearing_ring,right:sample_bottle"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("allowed_start_tolerance", default_value="0.01"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gz_share, "launch", "sim_gazebo.launch.py"])),
            launch_arguments={"world": LaunchConfiguration("world"),
                              "grasp_targets": LaunchConfiguration("grasp_targets"),
                              "lidar": LaunchConfiguration("lidar"),
                              "gui": LaunchConfiguration("gui"),
                              "fix_base": LaunchConfiguration("fix_base"),
                              "spawn_delay": LaunchConfiguration("spawn_delay")}.items()),
        move_group,
        rviz,
    ])
