"""sim_gazebo.launch.py  --  Moz1 in Gazebo FORTRESS (gz-sim) via ros_gz.

Starts: ign gazebo (moz1_pickplace world), robot_state_publisher, spawns the
robot (ros_gz_sim `create`), the ros_gz_bridge (gz<->ROS for /cmd_vel, /odom,
/tf, /livox/*, /clock), and the ros2_control spawners (joint_state_broadcaster +
the four MoveIt controllers, native via ign_ros2_control).

Contract is identical to Isaac/real; pair with sim_gazebo_nav / sim_gazebo_moveit
which reuse the UNCHANGED moz1_navigation_bringup / moz1_moveit_config.

Args:
  world      gz-sim world file (default: moz1_pickplace.world, has fixtures)
  lidar      true/false — enable the gpu_lidar. Needs a working GPU render
             engine; set false on machines whose GPU can't drive ogre2 (the
             lidar would otherwise stall the sim). Control/base/arm still work.
"""
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _inject_default_inertials(urdf_text):
    """Add a default <inertial> to every link that has visual/collision geometry
    but no inertial. Gazebo's urdf2sdf DROPS inertial-less links (and the joint
    above them), which cascades into missing arms/controllers.

    This makes the sim robust to `moz1_description` losing inertials — which has
    happened repeatedly via robot-sync merges / pulls that revert the arm/gripper
    inertials. Links that ALREADY have an inertial (e.g. the 40 kg chassis and the
    parametrized base/leg/waist links) are left untouched. Pure frame links
    (base_link, world, neck) have no geometry and are skipped — Gazebo lumps them.
    Inertials are physics-only; RViz/MoveIt/the real robot ignore them."""
    root = ET.fromstring(urdf_text)
    added = []
    for link in root.findall("link"):
        if link.find("inertial") is not None:
            continue
        if link.find("visual") is None and link.find("collision") is None:
            continue
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "0.3"})
        ET.SubElement(inertial, "inertia", {"ixx": "0.001", "ixy": "0", "ixz": "0",
                                            "iyy": "0.001", "iyz": "0", "izz": "0.001"})
        added.append(link.get("name"))
    if added:
        print(f"[sim_gazebo] injected default inertials into {len(added)} link(s): "
              f"{', '.join(added)}")
    return ET.tostring(root, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("moz1_sim_gazebo")
    xacro_file = os.path.join(pkg, "config", "moz1_gazebo.urdf.xacro")
    world = LaunchConfiguration("world").perform(context)
    lidar = LaunchConfiguration("lidar").perform(context)  # 'true' / 'false'
    gui = LaunchConfiguration("gui").perform(context)      # 'true' / 'false'
    fix_base = LaunchConfiguration("fix_base").perform(context)  # 'true' / 'false'
    spawn_delay = float(LaunchConfiguration("spawn_delay").perform(context))

    # Run the xacro CLI (not xacro.process_file, whose mappings don't reliably
    # override an <xacro:arg> here). lidar:= toggles the sensor; sim_collision:=true
    # swaps the STL collision meshes for fast AABB boxes (Gazebo physics only).
    robot_desc = subprocess.check_output(
        ["xacro", xacro_file, f"lidar:={lidar}", f"fix_base:={fix_base}",
         "sim_collision:=true"], text=True)
    # Guarantee every geometry link has an inertial (moz1_description syncs keep
    # reverting the arm/gripper inertials → Gazebo drops the arms otherwise).
    robot_desc = _inject_default_inertials(robot_desc)

    # gz needs its own search paths (it doesn't read ROS's package:// or ament):
    #  * plugin lib dir so it finds libign_ros2_control-system.so
    #  * resource path (the share PARENT) so model://moz1_description/... resolves
    ros_lib = os.path.join(get_package_prefix("ign_ros2_control"), "lib")
    desc_parent = os.path.dirname(get_package_share_directory("moz1_description"))
    gz_env = {
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH":
            ros_lib + os.pathsep + os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
        "IGN_GAZEBO_RESOURCE_PATH":
            desc_parent + os.pathsep + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),

        # Force software rendering
        "LIBGL_ALWAYS_SOFTWARE": "1",
    }

    # Fortress: `ign gazebo`. -r run, -v3 verbose, -s server-only (headless).
    # gui:=false runs headless — needed on GPUs that can't render the 3D view
    # (otherwise the failing GUI renderer drags the sim during startup).
    gz_cmd = ["ign", "gazebo", "-r", "-v", "3", world]
    if gui.lower() in ("false", "0", "no"):
        gz_cmd.insert(2, "-s")
    gz = ExecuteProcess(cmd=gz_cmd, output="screen", additional_env=gz_env)

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen",
               parameters=[{"use_sim_time": True, "robot_description": robot_desc}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 # z = wheel radius (~0.105 m) + margin, so the base rests ON the
                 # ground, not jammed INTO it. Spawning at 0.05 buried the wheels +
                 # 40 kg chassis box ~5 cm underground → DART fought a huge static
                 # contact force every step → sim ran ~1 step / 10 s and the
                 # controllers couldn't activate (only shows with the free base).
                 arguments=["-topic", "robot_description", "-name", "moz1", "-z", "0.12"])

    # gz <-> ROS bridge. '[' = gz->ROS, ']' = ROS->gz, '@' = bidirectional.
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=[
                      "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                      "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                      "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                      "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                      # gz gpu_lidar publishes the PointCloudPacked on <topic>/points
                      # (the bare <topic> carries a LaserScan). Bridge the /points
                      # one and remap it back to the /livox/lidar ROS contract below.
                      "/livox/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
                      "/livox/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                      # camera mappings
                      "/cam_high/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/cam_high/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/cam_high/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                      "/cam_left_wrist/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/cam_left_wrist/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/cam_left_wrist/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                      "/cam_right_wrist/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/cam_right_wrist/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                      "/cam_right_wrist/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                      # grasp weld: ROS Empty -> gz DetachableJoint attach/detach
                      # (see the robot xacro + scripts/grasp_helper.py).
                      "/grasp/left/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
                      "/grasp/left/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
                      "/grasp/right/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
                      "/grasp/right/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
                  ],
                  remappings=[("/livox/lidar/points", "/livox/lidar")],
                  parameters=[{"use_sim_time": True}])

    # Sim-only grasp emulation: welds the object to the gripper on close, releases
    # on open (so the manipulation skill stays sim/real-agnostic). See the module.
    grasp_helper = Node(package="moz1_sim_gazebo", executable="grasp_helper.py",
                        name="grasp_helper", output="screen",
                        parameters=[{"use_sim_time": True}])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"],
                    output="screen")

    jsb = spawner("joint_state_broadcaster")
    controllers = [spawner(c) for c in (
        "torso_controller",  # holds the leg/waist upright (else the upper body slumps)
        "left_arm_controller", "right_arm_controller",
        "left_gripper_controller", "right_gripper_controller")]

    # order: robot spawned -> wait spawn_delay (sim needs time to load the CAD
    # collision meshes + start stepping, esp. CPU-only) -> joint_state_broadcaster
    # -> arm/gripper controllers. Without the wait the switch_controller calls fire
    # before the sim is stepping and time out.
    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn, on_exit=[TimerAction(period=spawn_delay, actions=[jsb])]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=controllers))

    return [gz, rsp, spawn, bridge, grasp_helper, after_spawn, after_jsb]


def generate_launch_description():
    pkg = get_package_share_directory("moz1_sim_gazebo")
    default_world = os.path.join(pkg, "worlds", "moz1_pickplace.world")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="gz-sim world (default has pick-place fixtures)."),
        DeclareLaunchArgument("lidar", default_value="true",
                              description="Enable gpu_lidar (needs a working GPU; "
                                          "set false if ogre2 can't init)."),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Run the gz GUI; set false for headless "
                                          "(needed on GPUs that can't render)."),
        DeclareLaunchArgument("fix_base", default_value="false",
                              description="Pin base to `world` (upright, stable — for "
                                          "MoveIt); false = free holonomic base."),
        DeclareLaunchArgument("spawn_delay", default_value="8.0",
                              description="Seconds to wait after spawn before starting "
                                          "the controllers (raise on slow/CPU-only "
                                          "machines so the sim is stepping first)."),
        OpaqueFunction(function=launch_setup),
    ])
