"""duo.launch.py  --  the Galbot G1 and the Spirit AI Moz1 in ONE Fortress world.

  ros2 launch hmlv_cell_gazebo duo.launch.py              # both robots + the duo demo
  ros2 launch hmlv_cell_gazebo duo.launch.py demo:=false  # the cell and both robots only

The G1 works the pallet-A side, the Moz1 the pallet-B side (scripts/duo_scene.py
lays the cell out, scripts/duo_demo.py runs it). Everything of a robot lives in its
own ROS namespace, /g1 and /moz1: controller manager, robot_state_publisher,
move_group, grasp_helper, the demo node, /<ns>/cmd_vel, /<ns>/odom — and its own TF
tree, /<ns>/tf (both robots have a base_link, odom, world). Neither robot package is
changed for this; the launch rewrites each robot description on the way in:

  * gz_ros2_control gets <ros><namespace>/<ns></namespace></ros>. All the plugin
    instances run in the one gz process and only the FIRST one to start passes its
    ROS arguments (rclcpp::init runs once) — so both point at one combined
    controllers file with fully qualified keys (/g1/controller_manager, ...);
  * the base's VelocityControl / OdometryPublisher topics become /<ns>/cmd_vel,
    /<ns>/odom, /<ns>/tf.

The G1 spawns at the origin (in front of pallet A); the Moz1 at the conveyor's
unload station, 14 mm further back so that its chassis front (0.321 m in base_link,
the G1's is 0.307 m) lines up with the cell laid out for the G1.

Args: demo, gui, rviz, speed, record (as transfer.launch.py), world_load_wait,
spawn_delay.
"""
import importlib.util
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

OUT_DIR = "/tmp/hmlv_duo"
TF_REMAP = [("/tf", "tf"), ("/tf_static", "tf_static")]


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _namespaced_controllers(files):
    """{ns: controllers.yaml} -> one params dict with every top-level key fully
    qualified (/<ns>/controller_manager, /<ns>/left_arm_controller, ...)."""
    out = {}
    for ns, path in files.items():
        with open(path) as f:
            for key, value in yaml.safe_load(f).items():
                out[f"/{ns}/{key}"] = value
    return out


def _namespace_urdf(urdf, ns, params_file):
    """gz_ros2_control into /<ns> with the combined params file; the base plugins'
    gz topics to /<ns>/cmd_vel, /<ns>/odom, /<ns>/tf."""
    robot = ET.fromstring(urdf)
    for plugin in robot.iter("plugin"):
        fname = plugin.get("filename", "")
        if "gz_ros2_control" in fname or "ign_ros2_control" in fname:
            for p in plugin.findall("parameters"):
                plugin.remove(p)
            ET.SubElement(plugin, "parameters").text = params_file
            ros = ET.SubElement(plugin, "ros")
            ET.SubElement(ros, "namespace").text = f"/{ns}"
        elif "velocity-control" in fname:
            plugin.find("topic").text = f"/{ns}/cmd_vel"
        elif "odometry-publisher" in fname:
            plugin.find("odom_topic").text = f"/{ns}/odom"
            plugin.find("tf_topic").text = f"/{ns}/tf"
    return ET.tostring(robot, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory("hmlv_cell_gazebo")
    lib = os.path.join(get_package_prefix("hmlv_cell_gazebo"), "lib", "hmlv_cell_gazebo")
    moz1_share = get_package_share_directory("moz1_sim_gazebo")
    moz1_launch = _load(os.path.join(moz1_share, "launch", "sim_gazebo.launch.py"),
                        "moz1_sim_gazebo_launch")
    g1_launch = _load(os.path.join(share, "launch", "g1_sim.launch.py"), "g1_sim_launch")
    profiles = {}
    for ns in ("g1", "moz1"):
        with open(os.path.join(share, "robots", f"{ns}.yaml")) as f:
            profiles[ns] = yaml.safe_load(f)
    profile_files = {ns: os.path.join(share, "robots", f"{ns}.yaml") for ns in profiles}

    # --- the cell (pallet A for the G1, the belt with four cans, pallet B's box)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(subprocess.check_output(
        [os.path.join(lib, "duo_scene.py"), "--out-dir", OUT_DIR], text=True).strip())
    world = os.path.join(OUT_DIR, "hmlv_duo.world")
    scene_file = os.path.join(OUT_DIR, "hmlv_duo.yaml")
    with open(scene_file) as f:
        scene = yaml.safe_load(f)
    grasp = {ns: [(t["side"], t["name"]) for t in scene["grasp_targets"][ns]] for ns in profiles}

    # --- one controllers file for both (see the module docstring)
    params_file = os.path.join(OUT_DIR, "controllers.yaml")
    with open(params_file, "w") as f:
        yaml.safe_dump(_namespaced_controllers({
            "g1": os.path.join(share, "robots", "g1", "controllers.yaml"),
            "moz1": os.path.join(moz1_share, "config", "gazebo_controllers.yaml")}), f)

    # --- robot descriptions
    urdf = {}
    g1_xacro = os.path.join(share, "robots", "g1", "galbot_g1_fortress.urdf.xacro")
    u = subprocess.check_output(["xacro", g1_xacro], text=True)
    u = g1_launch.use_mjcf_actuator_limits(
        u, get_package_share_directory("galbot_one_golf_description"))

    urdf["g1"] = _namespace_urdf(moz1_launch._inject_grasp_welds(u, grasp["g1"]), "g1",
                                 params_file)
    u = subprocess.check_output(
        ["xacro", os.path.join(moz1_share, "config", "moz1_gazebo.urdf.xacro"),
         "lidar:=false", "fix_base:=false", "sim_collision:=true"], text=True)
    u = moz1_launch._inject_default_inertials(u)
    urdf["moz1"] = _namespace_urdf(moz1_launch._inject_grasp_welds(u, grasp["moz1"]),
                                   "moz1", params_file)

    # --- gz
    gz_env = {
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": os.pathsep.join([
            os.path.join(get_package_prefix("gz_ros2_control"), "lib"),
            os.path.join(get_package_prefix("moz1_sim_gazebo"), "lib"),
            os.path.join(get_package_prefix("hmlv_cell_gazebo"), "lib"),
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")]),
        "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join([
            os.path.dirname(get_package_share_directory("moz1_description")),
            os.path.dirname(get_package_share_directory("galbot_one_golf_description")),
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]),
        "LIBGL_ALWAYS_SOFTWARE": "1",
    }
    gz_cmd = ["ign", "gazebo", "-r", "-v", "3", world]
    if arg("gui").lower() in ("false", "0", "no"):
        gz_cmd.insert(2, "-s")
    nodes = [ExecuteProcess(cmd=gz_cmd, output="screen", additional_env=gz_env)]

    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                   "/conveyor/start@std_msgs/msg/Empty]ignition.msgs.Empty",
                   "/conveyor/done@std_msgs/msg/Empty[ignition.msgs.Empty"]
    for ns in profiles:
        bridge_args += [f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                        f"/{ns}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                        f"/{ns}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"]
        bridge_args += [f"/grasp/{side}/{model}/{act}@std_msgs/msg/Empty]ignition.msgs.Empty"
                        for side, model in grasp[ns] for act in ("attach", "detach")]
    nodes.append(Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                      arguments=bridge_args, parameters=[{"use_sim_time": True}]))

    # --- per robot
    moveit = {"g1": ("galbot_g1", "galbot_g1_moveit_config"),
              "moz1": ("moz1", "moz1_moveit_config")}
    spawn_pose = {"g1": (0.0, 0.0, g1_launch.SPAWN_Z),
                  "moz1": (profiles["g1"]["scene"]["base_front"]
                           - profiles["moz1"]["scene"]["base_front"],
                           scene["stations"]["unload"][1], 0.12)}
    model_name = {"g1": "galbot_g1", "moz1": "moz1"}
    controllers = {"g1": ["leg_controller", "head_controller"],
                   "moz1": ["torso_controller"]}
    demo_nodes, spawns, spawners = [], {}, {}
    for ns, prof in profiles.items():
        nodes.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                          namespace=ns, output="screen", remappings=TF_REMAP,
                          parameters=[{"use_sim_time": True, "robot_description": urdf[ns]}]))
        x, y, z = spawn_pose[ns]
        spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                     arguments=["-topic", f"/{ns}/robot_description", "-name", model_name[ns],
                                "-x", str(x), "-y", str(y), "-z", str(z)])
        spawner = Node(package="controller_manager", executable="spawner", output="screen",
                       arguments=["joint_state_broadcaster", *controllers[ns],
                                  "left_arm_controller", "right_arm_controller",
                                  "left_gripper_controller", "right_gripper_controller",
                                  "--controller-manager", f"/{ns}/controller_manager",
                                  "--controller-manager-timeout", "30"])
        spawns[ns], spawners[ns] = spawn, spawner
        g = prof["gripper"]
        # grasp_helper (moz1_sim_gazebo) names its topics absolutely: remap them in
        remaps = [("/joint_states", f"/{ns}/joint_states")] + [
            (f"/grasp/{s}/{t}", f"/{ns}/grasp/{s}/{t}")
            for s in ("left", "right") for t in ("target", "attach", "detach")]
        nodes.append(Node(
            package="moz1_sim_gazebo", executable="grasp_helper.py", name="grasp_helper",
            namespace=ns, output="screen", remappings=remaps,
            parameters=[{"use_sim_time": True,
                         "closed_threshold": float(g["closed_threshold"]),
                         "open_threshold": float(g["open_threshold"]),
                         "detach_on_open": bool(g.get("detach_on_open", True)),
                         "left_targets": [m for s, m in grasp[ns] if s == "left"] or [""],
                         "right_targets": [m for s, m in grasp[ns] if s == "right"] or [""]}]))
        robot, pkg = moveit[ns]
        mc = MoveItConfigsBuilder(robot, package_name=pkg) \
            .planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
        nodes.append(Node(
            package="moveit_ros_move_group", executable="move_group", namespace=ns,
            output="screen", remappings=TF_REMAP,
            parameters=[mc.to_dict(), {"use_sim_time": True,
                                       "trajectory_execution.allowed_start_tolerance": 0.05}]))
        demo_nodes.append(Node(
            package="hmlv_cell_gazebo", executable="duo_demo.py", name="duo_demo",
            namespace=ns, output="screen", remappings=TF_REMAP, arguments=[ns],
            condition=IfCondition(arg("demo")),
            parameters=[{"use_sim_time": True, "scene_file": scene_file,
                         "robot_file": profile_files[ns], "speed": float(arg("speed"))}]))
    # One robot at a time: the G1 spawned (once the world has loaded, see
    # g1_sim.launch.py) and its controllers up, THEN the Moz1 spawned and its
    # controllers up. Both run in the one gz process, and at the same time
    #  * two gz_ros2_control instances fetching robot_description at once: one
    #    fetch timed out ("failed to send response ... (timeout)") and that robot's
    #    controller manager never came up;
    #  * two controller managers loading the same controller library at once hit a
    #    pluginlib race ("... explicitly loaded through MultiLibraryClassLoader::
    #    loadLibrary()") — torso_controller failed to load.
    nodes.append(TimerAction(period=float(arg("world_load_wait")), actions=[spawns["g1"]]))
    for ns in ("g1", "moz1"):
        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=spawns[ns],
            on_exit=[TimerAction(period=float(arg("spawn_delay")), actions=[spawners[ns]])])))
    nodes.append(RegisterEventHandler(OnProcessExit(
        target_action=spawners["g1"], on_exit=[spawns["moz1"]])))
    nodes += demo_nodes
    # the cans' poses on the global /tf (gz_world -> jerrycan_*), for the recording
    nodes.append(Node(
        package="moz1_sim_gazebo", executable="gz_pose_tf", name="gz_pose_tf", output="screen",
        parameters=[{"use_sim_time": True, "world_name": scene["world_name"],
                     "model_prefix": "jerrycan_", "parent_frame": "gz_world"}]))
    if arg("record"):
        nodes.append(Node(
            package="hmlv_cell_gazebo", executable="record_run.py", name="record_run",
            output="screen",
            parameters=[{"use_sim_time": True, "out_dir": arg("record"), "world_file": world,
                         "robot": "duo", "robots": list(profiles)}]))
    nodes.append(Node(package="hmlv_cell_gazebo", executable="duo_demo.py",
                      name="duo_supervisor", output="screen", arguments=["supervisor"],
                      condition=IfCondition(arg("demo")),
                      parameters=[{"use_sim_time": True, "scene_file": scene_file}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("demo", default_value="true"),
        DeclareLaunchArgument("speed", default_value="0.4"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("world_load_wait", default_value="6.0"),
        DeclareLaunchArgument("spawn_delay", default_value="8.0"),
        OpaqueFunction(function=launch_setup),
    ])
