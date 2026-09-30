"""sim_gazebo_jerrycan.launch.py  --  Moz1 depalletising jerry cans in Gazebo.

One command for the jerry-can picking demo:

  1. generates the cell (scripts/jerrycan_scene.py): a EUR pallet stacked with
     TCCC jerry cans of the chosen size, and an infeed conveyor on each side;
  2. starts Gazebo + MoveIt (sim_gazebo_moveit.launch.py) with the base pinned and
     a grasp weld for every pickable can;
  3. runs scripts/jerrycan_demo.py, which moves the top-front row of cans from
     the pallet onto the conveyors, alternating arms.

  ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py
  ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py container:=10 arms:=left
  ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py demo:=false   # scene only

Args:
  container  4 | 10 | 20      jerry can size in litres (default 4)
  layers     stack levels, 0 = size default (3 for 4/10 L, 2 for 20 L)
  density    liquid density in g/mL (default 1.0); sets the can mass
  demo       true/false — run the pick-and-place sequence (default true)
  arms       both | left | right (default both)
  count      max cans to move, 0 = all (default 0)
  speed      free-space velocity scaling 0..1 (default 0.4)
  gui        true/false — Gazebo's own 3D window (default FALSE: gz runs headless
             and RViz is the view, like artc_ranger_xarm6; the gz window renders
             black on some GPUs)
  rviz       true/false — RViz with the robot, the cell and MoveIt's planned path
  lidar, spawn_delay   passed to the sim (lidar defaults to false here: the arm
             demo doesn't need it and it costs GPU time)

RViz view: the cell is mirrored from the world SDF by scripts/scene_markers.py;
the jerry cans follow the live simulation through src/gz_pose_tf.cc, which
publishes each can's Gazebo pose as a TF frame (gz_world -> jerrycan_*).

The generated world + scene YAML land in /tmp/moz1_jerrycan_<container>L/.
"""
import os
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    container = arg("container")
    out_dir = f"/tmp/moz1_jerrycan_{container}L"
    gen = os.path.join(get_package_prefix("moz1_sim_gazebo"), "lib", "moz1_sim_gazebo",
                       "jerrycan_scene.py")
    print(subprocess.check_output(
        [gen, "--container", container, "--layers", arg("layers"), "--density", arg("density"),
         "--step", arg("physics_step"),
         "--out-dir", out_dir], text=True).strip())
    world = os.path.join(out_dir, "moz1_jerrycan.world")
    scene_file = os.path.join(out_dir, "moz1_jerrycan.yaml")
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    grasp_targets = ",".join(f"{t['side']}:{t['name']}" for t in scene["targets"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("moz1_sim_gazebo"), "launch",
            "sim_gazebo_moveit.launch.py")),
        launch_arguments={"world": world, "grasp_targets": grasp_targets, "fix_base": "true",
                          "gui": arg("gui"), "rviz": "false", "lidar": arg("lidar"),
                          "spawn_delay": arg("spawn_delay"),
                          # a 4 kg can makes the arm sag ~0.02 rad under gz's
                          # position control; MoveIt's 0.01 default rejects that
                          "allowed_start_tolerance": "0.05"}.items())

    # --- RViz view of the cell (Gazebo can stay headless) ---
    # gz world vs. TF `world`: with the base pinned the robot model sits at
    # z = 0.01 in gz (sim_gazebo.launch.py), and TF `world` is that model's origin.
    gz_world_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="world_to_gz_world",
        arguments=["--frame-id", "world", "--child-frame-id", "gz_world", "--z", "-0.01"],
        parameters=[{"use_sim_time": True}])
    can_tf = Node(
        package="moz1_sim_gazebo", executable="gz_pose_tf", name="gz_pose_tf", output="screen",
        parameters=[{"use_sim_time": True, "world_name": scene["world_name"],
                     "model_prefix": "jerrycan_", "parent_frame": "gz_world"}])
    markers = Node(
        package="moz1_sim_gazebo", executable="scene_markers.py", name="scene_markers",
        output="screen",
        parameters=[{"use_sim_time": True, "world_file": world, "frame_id": "gz_world",
                     # the pick targets move: draw them in their live TF frames
                     "tf_model_prefix": "jerrycan_"}])
    moveit_config = MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config") \
        .planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        condition=IfCondition(arg("rviz")),
        arguments=["-d", os.path.join(get_package_share_directory("moz1_sim_gazebo"),
                                      "config", "moz1_jerrycan.rviz")],
        parameters=[moveit_config.robot_description,           # for the path display
                    moveit_config.robot_description_semantic, {"use_sim_time": True}])

    demo = Node(package="moz1_sim_gazebo", executable="jerrycan_demo.py",
                name="jerrycan_demo", output="screen",
                condition=IfCondition(arg("demo")),
                parameters=[{"use_sim_time": True, "scene_file": scene_file,
                             "arms": arg("arms"), "count": int(arg("count")),
                             "speed": float(arg("speed"))}])
    return [sim, gz_world_tf, can_tf, markers, rviz, demo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("container", default_value="4",
                              description="Jerry can size in litres: 4, 10 or 20."),
        DeclareLaunchArgument("layers", default_value="0",
                              description="Stack levels; 0 = default for the size."),
        DeclareLaunchArgument("density", default_value="1.0",
                              description="Liquid density, g/mL."),
        DeclareLaunchArgument("physics_step", default_value="0.004",
                              description="gz physics step, s."),
        DeclareLaunchArgument("demo", default_value="true",
                              description="Run the pick-and-place sequence."),
        DeclareLaunchArgument("arms", default_value="both",
                              description="both | left | right"),
        DeclareLaunchArgument("count", default_value="0",
                              description="Max cans to move; 0 = all targets."),
        DeclareLaunchArgument("speed", default_value="0.4",
                              description="Free-space velocity scaling, 0..1."),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Gazebo's own window; RViz is the view."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("lidar", default_value="false"),
        DeclareLaunchArgument("spawn_delay", default_value="8.0"),
        OpaqueFunction(function=launch_setup),
    ])
