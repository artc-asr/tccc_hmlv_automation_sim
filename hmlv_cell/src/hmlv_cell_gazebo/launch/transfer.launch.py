"""transfer.launch.py  --  a dual-arm robot moves a pair of jerry cans through a filling line.

  ros2 launch hmlv_cell_gazebo transfer.launch.py                 # Spirit AI Moz1
  ros2 launch hmlv_cell_gazebo transfer.launch.py robot:=g1       # Galbot G1
  ros2 launch hmlv_cell_gazebo transfer.launch.py demo:=false     # scene only

One pair, 4 L cans: both arms pick two EMPTY cans (0.2 kg) from pallet A (leaning
the torso over it if needed), the base drives to the conveyor, the cans ride to
the filling station and become 4.2 kg, the robot drives there, picks both, drives
to pallet B and places them on its deck, nearest row. See scripts/transfer_demo.py.

  1. scripts/transfer_scene.py generates the cell into /tmp/hmlv_transfer_<robot>/,
     laid out from the robot's chassis front (robots/<robot>.yaml);
  2. the robot's Fortress sim + move_group, base FREE (it drives on /cmd_vel), a
     grasp plugin per pickable can, and the ConveyorBelt world plugin:
     moz1 -> moz1_sim_gazebo's sim_gazebo_moveit.launch.py,
     g1   -> g1_sim.launch.py (Galbot G1 in Fortress, galbot_g1_moveit_config);
  3. RViz is the view (Gazebo headless unless gui:=true), like the jerry-can demo;
  4. scripts/transfer_demo.py with the robot's profile.

Args:
  robot         moz1 (default) | g1
  cleared_rows  top-layer rows of pallet A already taken (default: the robot
                profile's scene.cleared_rows, else 4 — the pair comes from row 5 and
                the torso has to lean over the pallet)
  layers        stack height on pallet A (default: scene.layers, else 3); the pair
                comes from its top layer, the furthest row with nothing in front
  The profile's scene.box_on_floor puts pallet B's box on the floor (no pallet B).
  demo          run the sequence (default true)
  speed         free-space velocity scaling (default 0.4)
  record        a directory: record the run there for the web replay
                (scripts/record_run.py; tools/web_replay/build.py builds docs/ from it)
  gui, rviz, spawn_delay
"""
import os
import subprocess
from pathlib import Path

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

    robot = arg("robot")
    share = get_package_share_directory("hmlv_cell_gazebo")
    robot_file = os.path.join(share, "robots", f"{robot}.yaml")
    if not os.path.isfile(robot_file):
        known = sorted(p.stem for p in Path(share, "robots").glob("*.yaml"))
        raise RuntimeError(f"robot:={robot}: no profile (known: {', '.join(known)})")
    with open(robot_file) as f:
        profile = yaml.safe_load(f)

    sc = profile["scene"]
    cleared_rows = arg("cleared_rows") or str(sc.get("cleared_rows", 4))
    layers = arg("layers") or str(sc.get("layers", 3))
    out_dir = f"/tmp/hmlv_transfer_{robot}"
    gen = os.path.join(get_package_prefix("hmlv_cell_gazebo"), "lib", "hmlv_cell_gazebo",
                       "transfer_scene.py")
    print(subprocess.check_output(
        [gen, "--cleared-rows", cleared_rows, "--layers", layers,
         "--base-front", str(sc["base_front"]), "--base-z", str(sc["base_z"]),
         "--out-dir", out_dir] + (["--box-on-floor"] if sc.get("box_on_floor") else []),
        text=True).strip())
    world = os.path.join(out_dir, "hmlv_transfer.world")
    scene_file = os.path.join(out_dir, "hmlv_transfer.yaml")
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    grasp_targets = ",".join(f"{t['side']}:{t['name']}" for t in scene["targets"])

    sim_args = {"world": world, "grasp_targets": grasp_targets, "gui": arg("gui"),
                "spawn_delay": arg("spawn_delay"), "allowed_start_tolerance": "0.05"}
    if robot == "moz1":
        sim_launch = os.path.join(get_package_share_directory("moz1_sim_gazebo"), "launch",
                                  "sim_gazebo_moveit.launch.py")
        sim_args.update({"fix_base": "false",           # the base drives between stations
                         "rviz": "false", "lidar": "false"})
    else:
        sim_launch = os.path.join(share, "launch", f"{robot}_sim.launch.py")
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch),
                                   launch_arguments=sim_args.items())

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
    mc = profile["moveit_config"]
    moveit_config = MoveItConfigsBuilder(mc["robot"], package_name=mc["package"]) \
        .planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        condition=IfCondition(arg("rviz")),
        arguments=["-d", os.path.join(share, "config", "transfer.rviz")],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic, {"use_sim_time": True}])

    demo = Node(package="hmlv_cell_gazebo", executable="transfer_demo.py",
                name="transfer_demo", output="screen",
                condition=IfCondition(arg("demo")),
                parameters=[{"use_sim_time": True, "scene_file": scene_file,
                             "robot_file": robot_file, "speed": float(arg("speed"))}])
    nodes = [sim, conveyor_bridge, *tfs, can_tf, markers, rviz, demo]
    if arg("record"):
        nodes.append(Node(
            package="hmlv_cell_gazebo", executable="record_run.py", name="record_run",
            output="screen",
            parameters=[{"use_sim_time": True, "out_dir": arg("record"), "world_file": world,
                         "robot": robot}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="moz1",
                              description="moz1 (Spirit AI Moz1) | g1 (Galbot G1)"),
        DeclareLaunchArgument("cleared_rows", default_value="",
                              description="Top-layer rows of pallet A already taken "
                                          "(default: the robot profile's)."),
        DeclareLaunchArgument("layers", default_value="",
                              description="Stack height on pallet A, pick from its top "
                                          "(default: the robot profile's)."),
        DeclareLaunchArgument("demo", default_value="true"),
        DeclareLaunchArgument("speed", default_value="0.4"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Gazebo's own window; RViz is the view."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("spawn_delay", default_value="8.0"),
        DeclareLaunchArgument("record", default_value="",
                              description="Directory to record the run into (web replay)."),
        OpaqueFunction(function=launch_setup),
    ])
