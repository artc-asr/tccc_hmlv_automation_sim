# ============================================================================
# moveit_rviz.launch.py - RViz2 with the MotionPlanning panel for Moz1.
# Loads config/moveit.rviz. Visualization only.
# ============================================================================
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
