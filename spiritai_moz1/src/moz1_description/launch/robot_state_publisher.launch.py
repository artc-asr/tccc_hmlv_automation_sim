# ============================================================================
# robot_state_publisher.launch.py
#
# Headless TF publisher for the Moz1 model. Starts ONLY:
#   * robot_state_publisher  (URDF -> /tf, /robot_description)
#   * joint_state_publisher  (publishes the home pose to /joint_states so TF is
#     fully defined even with no GUI and no real robot)
#
# This launch NEVER talks to hardware. It is a pure visualization/TF helper,
# reusable as an include from other launch files.
#
# Args:
#   use_jsp (default true) : also start joint_state_publisher with the home pose.
#                            Set false if another node (e.g. a real-robot state
#                            bridge) will publish /joint_states instead.
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("moz1_description")
    xacro_file = os.path.join(pkg, "urdf", "moz1.urdf.xacro")
    home_file = os.path.join(pkg, "config", "home_joint_states.yaml")

    use_jsp = LaunchConfiguration("use_jsp")
    visual_yaw = LaunchConfiguration("visual_yaw")

    # Expand xacro at launch time so edits to the .xacro take effect on relaunch.
    # visual_yaw is passed through to the URDF's <xacro:arg name="visual_yaw">,
    # rotating ONLY the visual chassis assembly (mesh + wheels + arms etc.)
    # relative to base_link -- nav-side frames are untouched.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " visual_yaw:=", visual_yaw,
        ]),
        value_type=str,
    )

    # Remap /joint_states to /moz1/joint_states so we never clash with
    # MovaX's MCMessageNode, which also publishes to /joint_states but with
    # a future-stamped clock (~70-100 s ahead). Both RSP and JSP are kept on
    # the private topic; nothing outside this launch needs to subscribe.
    joint_states_remap = [("/joint_states", "/moz1/joint_states")]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_jsp", default_value="true",
            description="Start joint_state_publisher with the home pose.",
        ),
        DeclareLaunchArgument(
            "visual_yaw", default_value="1.5707963267948966",
            description="Yaw (rad) applied to the URDF's visual chassis "
                        "assembly so it aligns with the localizer pose arrow. "
                        "Default +pi/2 (+90 deg, CCW): the CAD mesh was "
                        "authored with its 'front' along base_link's +Y, so a "
                        "+pi/2 yaw rotates the mesh assembly so its front "
                        "points along +X (the conventional 'red arrow' "
                        "direction in RViz). Empirically verified against the "
                        "live robot. Override with visual_yaw:=<value> if your "
                        "convention differs (e.g. 0.0 to disable, 3.14159 for "
                        "180 deg, -1.5708 for -pi/2).",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": False}],
            remappings=joint_states_remap,
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=IfCondition(use_jsp),
            parameters=[home_file, {"use_sim_time": False}],
            remappings=joint_states_remap,
        ),
    ])
