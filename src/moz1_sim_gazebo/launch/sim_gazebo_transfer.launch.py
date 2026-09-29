"""sim_gazebo_transfer.launch.py  --  Moz1 moves a pair of jerry cans through a filling line.

  ros2 launch moz1_sim_gazebo sim_gazebo_transfer.launch.py
  ros2 launch moz1_sim_gazebo sim_gazebo_transfer.launch.py demo:=false   # scene only

One pair, 4 L cans: both arms pick two EMPTY cans (0.2 kg) from pallet A (leaning
the torso over it if needed), the base drives to the conveyor, the cans ride to
the filling station and become 4.2 kg, the robot drives there, picks both, drives
to pallet B and places them on its deck, nearest row. See scripts/transfer_demo.py.

  1. scripts/transfer_scene.py generates the cell into /tmp/moz1_transfer/;
  2. sim_gazebo_moveit.launch.py with the base FREE (it drives on /cmd_vel), a
     grasp plugin per pickable can, and the ConveyorBelt world plugin;
  3. RViz is the view (Gazebo headless unless gui:=true), like the jerry-can demo.

Args:
  cleared_rows  top-layer rows of pallet A already taken (default 2, so the pair
                comes from row 3 and the torso has to lean over the pallet)
  demo          run the sequence (default true)
  speed         free-space velocity scaling (default 0.4)
  gui, rviz, spawn_delay
"""
import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    out_dir = "/tmp/moz1_transfer"
    gen = os.path.join(get_package_prefix("moz1_sim_gazebo"), "lib", "moz1_sim_gazebo",
                       "transfer_scene.py")
    print(subprocess.check_output(
        [gen, "--cleared-rows", arg("cleared_rows"), "--out-dir", out_dir], text=True).strip())
    world = os.path.join(out_dir, "moz1_transfer.world")
    scene_file = os.path.join(out_dir, "moz1_transfer.yaml")
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    grasp_targets = ",".join(f"{t['side']}:{t['name']}" for t in scene["targets"])
    share = get_package_share_directory("moz1_sim_gazebo")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch",
                                                   "sim_gazebo_moveit.launch.py")),
        launch_arguments={"world": world, "grasp_targets": grasp_targets,
                          "fix_base": "false",           # the base drives between stations
                          "gui": arg("gui"), "rviz": "false", "lidar": "false",
                          "spawn_delay": arg("spawn_delay"),
                          "allowed_start_tolerance": "0.05"}.items())

    # the conveyor's start/done topics (ConveyorBelt plugin in the world)
    conveyor_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="conveyor_bridge",
        output="screen",
        arguments=["/conveyor/start@std_msgs/msg/Empty]ignition.msgs.Empty",
                   "/conveyor/done@std_msgs/msg/Empty[ignition.msgs.Empty"],
        parameters=[{"use_sim_time": True}])

    # --- RViz view. Free base: odom (= the gz world, the base spawns at its origin)
    # -> base_link comes from gz. MoveIt's `world` is the base itself (fixed SRDF
    # virtual joint), so it hangs off base_link for the planned-path display.
    tfs = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="odom_to_gz_world",
             arguments=["--frame-id", "odom", "--child-frame-id", "gz_world"],
             parameters=[{"use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_to_world",
             arguments=["--frame-id", "base_link", "--child-frame-id", "world"],
             parameters=[{"use_sim_time": True}]),
    ]
    can_tf = Node(
        package="moz1_sim_gazebo", executable="gz_pose_tf", name="gz_pose_tf", output="screen",
        parameters=[{"use_sim_time": True, "world_name": scene["world_name"],
                     "model_prefix": "jerrycan_", "parent_frame": "gz_world"}])
    markers = Node(
        package="moz1_sim_gazebo", executable="scene_markers.py", name="scene_markers",
        output="screen",
        parameters=[{"use_sim_time": True, "world_file": world, "frame_id": "gz_world",
                     "tf_model_prefix": "jerrycan_"}])
    moveit_config = MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config") \
        .planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        condition=IfCondition(arg("rviz")),
        arguments=["-d", os.path.join(share, "config", "moz1_transfer.rviz")],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic, {"use_sim_time": True}])

    demo = Node(package="moz1_sim_gazebo", executable="transfer_demo.py",
                name="transfer_demo", output="screen",
                condition=IfCondition(arg("demo")),
                parameters=[{"use_sim_time": True, "scene_file": scene_file,
                             "speed": float(arg("speed"))}])
    return [sim, conveyor_bridge, *tfs, can_tf, markers, rviz, demo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cleared_rows", default_value="2",
                              description="Top-layer rows of pallet A already taken."),
        DeclareLaunchArgument("demo", default_value="true"),
        DeclareLaunchArgument("speed", default_value="0.4"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Gazebo's own window; RViz is the view."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("spawn_delay", default_value="8.0"),
        OpaqueFunction(function=launch_setup),
    ])
