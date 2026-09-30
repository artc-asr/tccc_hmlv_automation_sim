# ============================================================================
# rviz.launch.py — RViz on its own, for stacks that don't start one.
#
#   ros2 launch moz1_moveit_config rviz.launch.py                  # model at home pose
#   ros2 launch moz1_moveit_config rviz.launch.py jsp:=false       # follow the LIVE robot
#   ros2 launch moz1_moveit_config rviz.launch.py rsp:=false jsp:=false   # attach to a stack
#
# WHY THIS EXISTS
#   moz1_moveit_bridge's trajectory_bridge.launch.py is HEADLESS BY DESIGN — it
#   starts one node that subscribes to /planned_trajectory and prints what it
#   would send. It has no RViz, no robot_state_publisher and no TF, so "nothing
#   happens" is the correct behaviour; it is waiting for a trajectory. Run this
#   alongside it to actually see the robot.
#
#   demo.launch.py and real.launch.py already start their own RViz — use
#   `use_rviz:=false` / `rviz:=false` there and this launch with
#   `rsp:=false jsp:=false` if you want the window managed separately.
#
# ARGS
#   rsp   true  start robot_state_publisher (URDF -> TF) + the world->base_link
#               static TF. false when another launch already publishes them.
#   jsp   true  start joint_state_publisher holding the home pose.
#               ⚠ Set FALSE whenever anything else publishes /joint_states —
#               MovaX's MCMessageNode does (verified 2026-08-17: same URDF joint
#               names, so with jsp:=false RViz follows the REAL robot live), as do
#               demo.launch.py and the moz1_bringup driver. Two publishers on one
#               topic give TF_OLD_DATA warnings and a model that flickers between
#               two poses.
#   config      moveit   config/moveit.rviz — MotionPlanning panel, Fixed Frame 'world'
#               display  moz1_description/rviz/moz1_display.rviz — RobotModel + TF
#   joint_states  /joint_states  remap; use /moz1/joint_states with the real driver.
#
# The MotionPlanning panel needs move_group. Without it (this launch alone, or
# next to the bridge) the panel loads but logs "Failed to fetch planning scene"
# and cannot plan — that is expected, not a fault. Start demo.launch.py or
# real.launch.py for planning.
#
# SAFETY: visualization only. No controllers, no SDK, no MovaX — cannot move the
# robot. The description is this package's mock xacro, so nothing here can be
# armed either.
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# ⚠ Snap contamination — the reason a bare `rviz2` dies in some terminals.
#
# Launched from a snap-packaged terminal (VS Code's integrated terminal is one),
# the environment carries GTK_EXE_PREFIX=/snap/code/... plus a GIO module cache
# of symlinks into /snap/code, whose libraries have an RPATH into /snap/core20.
# GLib then loads snap's libpthread into rviz2 and it dies immediately with
#   symbol lookup error: /snap/core20/.../libpthread.so.0:
#   undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
# exiting 127 — which reads like "rviz2 is not installed". It is installed.
# Re-measured 2026-08-17: bare `rviz2 --help` exits 127 in that terminal and 0
# with these overrides. Same fix as real.launch.py; see docs/troubleshooting.md.
SYSTEM_GTK_ENV = {
    "GTK_EXE_PREFIX": "/usr",
    "GTK_PATH": "",
    "GTK_MODULES": "",
    "GIO_MODULE_DIR": "/usr/lib/x86_64-linux-gnu/gio/modules",
    "GSETTINGS_SCHEMA_DIR": "/usr/share/glib-2.0/schemas",
    "GDK_PIXBUF_MODULE_FILE":
        "/usr/lib/x86_64-linux-gnu/gdk-pixbuf-2.0/2.10.0/loaders.cache",
    "XDG_DATA_HOME": os.path.expanduser("~/.local/share"),
}


def _setup(context, *args, **kwargs):
    moveit_share = get_package_share_directory("moz1_moveit_config")
    desc_share = get_package_share_directory("moz1_description")

    which = LaunchConfiguration("config").perform(context)
    if which == "display":
        rviz_file = os.path.join(desc_share, "rviz", "moz1_display.rviz")
    else:
        rviz_file = os.path.join(moveit_share, "config", "moveit.rviz")

    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Remap every node together: robot_state_publisher SUBSCRIBES to joint_states
    # and RViz's RobotModel/PlanningScene monitors do too, so a half-applied
    # remap shows a robot frozen at the origin instead of an error.
    js_remap = [("/joint_states", LaunchConfiguration("joint_states"))]

    nodes = [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            additional_env=SYSTEM_GTK_ENV,
            remappings=js_remap,
            arguments=["-d", rviz_file],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
    ]

    # --- rsp: enough TF for RViz to draw something ---
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        condition=IfCondition(LaunchConfiguration("rsp")),
        remappings=js_remap,
        parameters=[moveit_config.robot_description],
    ))

    # --- jsp: the home pose, when nothing else feeds /joint_states ---
    # Values come from config/robot.yaml via scripts/sync_home_pose.py — never
    # hand-edit home_joint_states.yaml (repo CLAUDE.md, "one source of truth").
    nodes.append(Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
        condition=IfCondition(LaunchConfiguration("jsp")),
        remappings=js_remap,
        parameters=[os.path.join(desc_share, "config", "home_joint_states.yaml")],
    ))

    # ⚠ child MUST be base_link — the URDF root, and what moz1.srdf's
    # <virtual_joint> names. Publishing world->base_footprint instead gives that
    # frame two parents, splits the tree, and moveit.rviz (Fixed Frame 'world')
    # then draws nothing at all. Same trap as real.launch.py, §12.20.
    nodes.append(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        condition=IfCondition(LaunchConfiguration("rsp")),
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rsp", default_value="true", choices=["true", "false"],
            description="Start robot_state_publisher (URDF -> TF) + the "
                        "world->base_link static TF.",
        ),
        DeclareLaunchArgument(
            "jsp", default_value="true", choices=["true", "false"],
            description="Start joint_state_publisher at the home pose. Set false "
                        "when anything else publishes /joint_states (MovaX, "
                        "demo.launch.py, the moz1_bringup driver) — with MovaX up, "
                        "jsp:=false makes RViz follow the real robot.",
        ),
        DeclareLaunchArgument(
            "config", default_value="moveit", choices=["moveit", "display"],
            description="moveit: MotionPlanning panel. display: RobotModel + TF.",
        ),
        DeclareLaunchArgument(
            "joint_states", default_value="/joint_states",
            description="Joint-state topic to remap onto "
                        "(/moz1/joint_states with the real driver).",
        ),
        OpaqueFunction(function=_setup),
    ])
