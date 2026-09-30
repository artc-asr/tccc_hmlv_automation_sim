"""g1_sim.launch.py  --  the Galbot G1 in Gazebo FORTRESS with move_group, for the cell.

The G1 counterpart of moz1_sim_gazebo's sim_gazebo_moveit.launch.py, which
transfer.launch.py includes for robot:=g1: `ign gazebo` with the given world, the
G1 (robots/g1/galbot_g1_fortress.urdf.xacro) on a free holonomic base, the
ros_gz bridge, one KinematicGrasp per grasp target + grasp_helper, the ros2_control
controllers, and move_group with galbot_g1_moveit_config.

Args:
  world          gz-sim world file (required)
  grasp_targets  side:model pairs the grippers can weld, comma separated
  gui            Gazebo's own window (default false: headless)
  spawn_delay    seconds between the spawn and the controller spawners
  allowed_start_tolerance   MoveIt's start-state tolerance (rad)
"""
import importlib.util
import json
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# base_link sits this high on the ground (galbot_g1_gazebo's spawn height); spawn a
# few mm above it and let it settle.
SPAWN_Z = 0.035


def _moz1_launch_helpers():
    """moz1_sim_gazebo's grasp-target parsing and grasp-plugin injection (they put
    one KinematicGrasp per (side, object) on <side>_gripper_base_link, which the
    G1 has too)."""
    path = os.path.join(get_package_share_directory("moz1_sim_gazebo"), "launch",
                        "sim_gazebo.launch.py")
    spec = importlib.util.spec_from_file_location("moz1_sim_gazebo_launch", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod._parse_grasp_targets, mod._inject_grasp_welds


def use_mjcf_actuator_limits(urdf, description_share):
    """Raise joint effort limits to the actuator force ranges of Galbot's MuJoCo
    model (as galbot_g1_gazebo does): the URDF's are too low for Gazebo's joint
    servos to hold the leg up under the upper body's weight."""
    with open(os.path.join(description_share, "config", "mjcf", "joint_data.json")) as f:
        joint_data = json.load(f)["joints"]
    robot = ET.fromstring(urdf)
    for joint in robot.iter("joint"):
        actuator = joint_data.get(joint.get("name"), {}).get("actuator", {})
        limit = joint.find("limit")
        if "forcerange" not in actuator or limit is None:
            continue
        effort = max(float(limit.get("effort", 0.0)), max(abs(v) for v in actuator["forcerange"]))
        limit.set("effort", str(effort))
    return ET.tostring(robot, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory("hmlv_cell_gazebo")
    description_share = get_package_share_directory("galbot_one_golf_description")
    with open(os.path.join(share, "robots", "g1.yaml")) as f:
        profile = yaml.safe_load(f)
    parse_targets, inject_welds = _moz1_launch_helpers()
    grasp_pairs = parse_targets(arg("grasp_targets"))

    urdf = subprocess.check_output(
        ["xacro", os.path.join(share, "robots", "g1", "galbot_g1_fortress.urdf.xacro")], text=True)
    urdf = inject_welds(use_mjcf_actuator_limits(urdf, description_share), grasp_pairs)

    gz_env = {
        # gz_ros2_control (apt, Fortress) and moz1_sim_gazebo's KinematicGrasp
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": os.pathsep.join([
            os.path.join(get_package_prefix("gz_ros2_control"), "lib"),
            os.path.join(get_package_prefix("moz1_sim_gazebo"), "lib"),
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")]),
        # model://galbot_one_golf_description/... (the STL collision meshes)
        "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join([
            os.path.dirname(description_share), os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]),
        "LIBGL_ALWAYS_SOFTWARE": "1",      # as moz1_sim_gazebo: no GPU sensors here
    }
    gz_cmd = ["ign", "gazebo", "-r", "-v", "3", arg("world")]
    if arg("gui").lower() in ("false", "0", "no"):
        gz_cmd.insert(2, "-s")
    gz = ExecuteProcess(cmd=gz_cmd, output="screen", additional_env=gz_env)

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"use_sim_time": True, "robot_description": urdf}])
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "galbot_g1",
                            "-z", str(SPAWN_Z)])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=[
                      "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                      "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                      "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                      "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                  ] + [f"/grasp/{side}/{model}/{act}@std_msgs/msg/Empty]ignition.msgs.Empty"
                       for side, model in grasp_pairs for act in ("attach", "detach")],
                  parameters=[{"use_sim_time": True}])
    g = profile["gripper"]
    grasp_helper = Node(package="moz1_sim_gazebo", executable="grasp_helper.py",
                        name="grasp_helper", output="screen",
                        parameters=[{"use_sim_time": True,
                                     "closed_threshold": float(g["closed_threshold"]),
                                     "open_threshold": float(g["open_threshold"]),
                                     # the demo detaches explicitly when it releases
                                     "detach_on_open": bool(g.get("detach_on_open", True)),
                                     "left_targets": [m for sd, m in grasp_pairs
                                                      if sd == "left"] or [""],
                                     "right_targets": [m for sd, m in grasp_pairs
                                                       if sd == "right"] or [""]}])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    jsb = spawner("joint_state_broadcaster")
    controllers = [spawner(c) for c in (
        "leg_controller",        # holds the upper body (else the leg folds)
        "head_controller", "left_arm_controller", "right_arm_controller",
        "left_gripper_controller", "right_gripper_controller")]
    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[TimerAction(period=float(arg("spawn_delay")), actions=[jsb])]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=controllers))

    moveit_config = (MoveItConfigsBuilder("galbot_g1", package_name="galbot_g1_moveit_config")
                     .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), {
            "use_sim_time": True,
            # an arm carrying a payload sags past MoveIt's default 0.01 rad
            "trajectory_execution.allowed_start_tolerance":
                float(arg("allowed_start_tolerance"))}])

    return [gz, rsp, spawn, bridge, grasp_helper, after_spawn, after_jsb, move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("grasp_targets", default_value=""),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("spawn_delay", default_value="8.0"),
        DeclareLaunchArgument("allowed_start_tolerance", default_value="0.05"),
        OpaqueFunction(function=launch_setup),
    ])
