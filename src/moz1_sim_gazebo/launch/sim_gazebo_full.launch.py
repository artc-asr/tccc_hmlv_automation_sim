"""sim_gazebo_full.launch.py  --  ONE Gazebo with BOTH navigation AND manipulation.

Closes the gap between sim_gazebo_nav (free base, drives) and sim_gazebo_moveit
(pinned base, arm planning): a single sim with the FREE holonomic base + lidar +
Nav2/SLAM *and* move_group, so the full pick-and-place DAG (navigate -> observe ->
check_reachability -> plan_grasp -> execute_grasp -> navigate -> place -> verify)
runs end-to-end in one world with the dashboard live.

  Drive it through the orchestrator backend with BOTH real backends:
      NAV_BACKEND=nav2 MANIP_BACKEND=skills python3 -m uvicorn webui.backend.app:app
      curl -X POST localhost:8000/mission/start -d '{"goal":"Pick up the bearing ring"}'

WHY THIS IS SUBTLE (the reason nav & manip were split): the MoveIt SRDF pins
base_link to `world` with a FIXED virtual joint ("mobile base is OUT of MoveIt
v1"). With a free base there is no `world` URDF link, so we publish a static
world->map identity TF; the tree becomes world->map->odom->base_link (SLAM owns
map->odom, gz owns odom->base_link). MoveIt treats base_link as fixed at the
world origin for arm planning — which is FINE because the DAG is SEQUENTIAL: the
base drives to a station and STOPS, then the arm plans while stationary. Nav and
manipulation never command the base at the same instant (the resource_arbiter
leases `base` for nav vs `arm`/`gripper` for manip). Arm joint trajectories are
relative to base_link, so they execute correctly wherever the base parked.
Caveat: don't expect a correct world-frame collision *scene* while the base is
mid-drive — that would need a planar/floating virtual joint (a moz1_moveit_config
change, out of scope here). For the sequential demo it isn't needed.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    gz_share = FindPackageShare("moz1_sim_gazebo")
    nav_share = FindPackageShare("moz1_navigation_bringup")
    nav2_bringup = FindPackageShare("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # SIM-ONLY nav param override (leaves the shared SpiritAI nav2_params.yaml —
    # used by the real robot — untouched). The shared file sets robot_radius=0.45
    # (a conservative placeholder; the real base is ~0.5 m across — wheels at
    # ±0.24 m), which keeps the base too far from the workbench for the arm to
    # reach a bin on it. Shrink it to the real footprint and tighten the goal
    # tolerance so the base parks right up to the counter for manipulation.
    sim_nav_params = RewrittenYaml(
        source_file=PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"]),
        param_rewrites={"robot_radius": "0.30", "xy_goal_tolerance": "0.15"},
        convert_types=True)

    # SRDF/kinematics/planners from moz1_moveit_config (unchanged); move_group
    # plans the arms, the Gazebo controllers execute — same as sim_gazebo_moveit,
    # but here the base is FREE so nav can drive it.
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # GUI ON by default — REQUIRED for the lidar on a hybrid Intel+NVIDIA
        # (PRIME) machine (headless gz won't render the gpu_lidar there). Same as
        # sim_gazebo_nav. Use RViz as the working view; minimise the gz window.
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("spawn_delay", default_value="20.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        SetParameter(name="use_sim_time", value=use_sim_time),

        # Sim: FREE base (fix_base:=false) so Nav2 can drive it, lidar on for SLAM.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_share, "launch", "sim_gazebo.launch.py"])),
            launch_arguments={"gui": LaunchConfiguration("gui"),
                              "fix_base": "false", "lidar": "true",
                              "spawn_delay": LaunchConfiguration("spawn_delay")}.items()),

        # Give MoveIt its `world` root without pinning the base: static world->map
        # identity. SLAM adds map->odom, gz adds odom->base_link -> single-parent
        # chain world->map->odom->base_link (see the module docstring).
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="world_to_map",
             arguments=["--frame-id", "world", "--child-frame-id", "map"],
             parameters=[{"use_sim_time": use_sim_time}], output="screen"),

        # Stage the nav stack + move_group only AFTER the sim + controllers are up
        # (spawn_delay=20). 30 s leaves margin; raise in lockstep with spawn_delay.
        TimerAction(period=30.0, actions=[
            Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
                 name="pointcloud_to_laserscan",
                 remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
                 parameters=[PathJoinSubstitution([nav_share, "config", "pointcloud_to_scan.yaml"]),
                             # horizontal-slab overrides (base_link frame): keep the
                             # walls/boxes, drop the floor ring — matches sim_gazebo_nav.
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
                                  "params_file": sim_nav_params,
                                  "use_composition": "False"}.items()),

            # move_group for the arms (base is free; see docstring on the fixed
            # virtual joint + sequential DAG).
            Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                 parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}]),

            # Single RViz: the nav view (RobotModel + map + scan + path + markers).
            # Pass the MoveIt params (SRDF + kinematics + pipelines) so a hand-added
            # **MotionPlanning** panel actually loads a planning scene — without them
            # it shows "No Planning Scene Loaded". (Same params the moveit launch's
            # RViz gets.) Add the panel via Panels → Add New Panel → MotionPlanning,
            # then pick planning group left_arm / right_arm to capture grasp poses.
            Node(package="rviz2", executable="rviz2", output="screen",
                 condition=IfCondition(LaunchConfiguration("rviz")),
                 arguments=["-d", PathJoinSubstitution(
                     [gz_share, "config", "moz1_nav.rviz"])],
                 parameters=[moveit_config.robot_description,
                             moveit_config.robot_description_semantic,
                             moveit_config.robot_description_kinematics,
                             moveit_config.planning_pipelines,
                             {"use_sim_time": use_sim_time}]),

            Node(package="moz1_sim_gazebo", executable="scene_markers.py",
                 name="scene_markers", output="screen",
                 condition=IfCondition(LaunchConfiguration("rviz")),
                 parameters=[{"use_sim_time": use_sim_time, "frame_id": "map",
                              "world_file": PathJoinSubstitution(
                                  [gz_share, "worlds", "moz1_pickplace.world"])}]),
        ]),
    ])
