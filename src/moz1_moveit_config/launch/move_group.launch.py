# ============================================================================
# move_group.launch.py - starts the MoveIt move_group node.
#
# This is the planning brain. With demo.launch.py it is paired with the mock
# ros2_control system, so any "execution" is purely simulated. move_group on its
# own NEVER commands hardware.
# ============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
