"""sim_gazebo_nav.launch.py  --  Gazebo Moz1 + the UNCHANGED Nav2/SLAM stack.

Same idea as moz1_sim_bringup/sim_nav.launch.py but backed by Gazebo instead of
Isaac. The nav pipeline is identical (it just consumes /livox/lidar + /odom):
  /livox/lidar -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> map->odom
  odom->base_link TF comes from Gazebo (planar_move, ground truth)
  Nav2 (MPPI Omni) -> /cmd_vel -> Gazebo planar_move (holonomic base)
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_share = FindPackageShare("moz1_sim_gazebo")
    nav_share = FindPackageShare("moz1_navigation_bringup")
    nav2_bringup = FindPackageShare("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # GUI ON by default — REQUIRED for the lidar on a hybrid Intel+NVIDIA
        # (PRIME) machine: the HEADLESS gz server does NOT render the scene for the
        # gpu_lidar there, so /scan comes back nearly empty (~2 returns) and SLAM
        # builds no map. The GUI provides the render context the sensor needs.
        # Controllers still activate fine (physics is fast after the spawn-height
        # fix). Use RViz as the main view; minimise the gz window. Set gui:=false
        # ONLY if your GPU renders headless sensors correctly.
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("spawn_delay", default_value="20.0"),
        # RViz is the view since the sim runs headless. rviz:=false to disable.
        DeclareLaunchArgument("rviz", default_value="true"),
        SetParameter(name="use_sim_time", value=use_sim_time),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_share, "launch", "sim_gazebo.launch.py"])),
            launch_arguments={"gui": LaunchConfiguration("gui"),
                              "spawn_delay": LaunchConfiguration("spawn_delay")}.items()),

        # Stage the nav stack: start pointcloud_to_laserscan + SLAM + Nav2 only
        # AFTER the sim and its controllers are up (spawn_delay=20). Bringing
        # Nav2(MPPI)+SLAM(Ceres) up at t=0 starves the gz-hosted controller
        # manager so the controllers never activate. 30 s leaves margin past
        # spawn_delay for activation; raise it in lockstep if you raise spawn_delay.
        TimerAction(period=30.0, actions=[
            Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
                 name="pointcloud_to_laserscan",
                 remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
                 parameters=[PathJoinSubstitution([nav_share, "config", "pointcloud_to_scan.yaml"]),
                             # sim overrides (base_link frame): carve a THIN
                             # HORIZONTAL SLAB at the lidar's height so only the
                             # walls/boxes' mid-section becomes the 2D scan. The old
                             # wide band [0.0, 2.0] let the downward-ray FLOOR RING
                             # through (nearest-per-angle -> circular map); combined
                             # with the horizontal-only lidar FOV, this slab keeps
                             # the scan on the walls. range_min 0.45 drops the
                             # robot's own torso (the base-mounted lidar sits near it).
                             {"use_sim_time": use_sim_time, "min_height": 0.15,
                              "max_height": 0.6, "range_min": 0.45}],
                 output="screen"),

            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
                [nav_share, "launch", "slam_mapping.launch.py"])),
                launch_arguments={"use_sim_time": use_sim_time,
                                  "slam_params": PathJoinSubstitution(
                                      [nav_share, "config", "slam_toolbox.yaml"])}.items()),

            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
                [nav2_bringup, "launch", "navigation_launch.py"])),
                launch_arguments={"use_sim_time": use_sim_time,
                                  "autostart": LaunchConfiguration("autostart"),
                                  "params_file": PathJoinSubstitution(
                                      [nav_share, "config", "nav2_params.yaml"]),
                                  "use_composition": "False"}.items()),

            Node(package="rviz2", executable="rviz2", output="screen",
                 condition=IfCondition(LaunchConfiguration("rviz")),
                 # sim-owned config: adds a RobotModel (see the robot) + /odom
                 # (sim odom, not the EKF /odometry/filtered) on top of the nav view.
                 arguments=["-d", PathJoinSubstitution(
                     [gz_share, "config", "moz1_nav.rviz"])],
                 parameters=[{"use_sim_time": use_sim_time}]),

            # Republish the gz world (tables/fixtures) as RViz markers so the scene
            # is visible with Gazebo headless. See scripts/scene_markers.py.
            Node(package="moz1_sim_gazebo", executable="scene_markers.py",
                 name="scene_markers", output="screen",
                 condition=IfCondition(LaunchConfiguration("rviz")),
                 parameters=[{"use_sim_time": use_sim_time, "frame_id": "map",
                              "world_file": PathJoinSubstitution(
                                  [gz_share, "worlds", "moz1_pickplace.world"])}]),
        ]),
    ])
