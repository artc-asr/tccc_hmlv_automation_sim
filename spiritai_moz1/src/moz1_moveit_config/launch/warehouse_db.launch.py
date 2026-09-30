# Optional MoveIt warehouse (motion DB). Off by default; only used with db:=true.
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_warehouse_db_launch(moveit_config)
