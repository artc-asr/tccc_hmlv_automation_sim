# ============================================================================
# demo.launch.py  -  Milestone-2 acceptance command (PRD §11):
#     ros2 launch moz1_moveit_config demo.launch.py
#
# Brings up the full FAKE-execution MoveIt stack:
#   * static world->base_footprint TF
#   * robot_state_publisher
#   * move_group (OMPL planning)
#   * RViz2 + MotionPlanning panel
#   * ros2_control_node with mock_components + controller spawners
#
# ----------------------------------------------------------------------------
# SAFETY: This launch is SIMULATION ONLY.
#   * The hardware interface is mock_components/GenericSystem - it loops
#     commands back to states. There is NO connection to the MozRobot SDK,
#     MovaX, or the 172.16.0.x motor bus. It CANNOT move the real robot.
#   * Real execution is intentionally a separate package (moz1_moveit_bridge)
#     and requires an explicit enable_motion:=true flag there.
# ----------------------------------------------------------------------------
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
