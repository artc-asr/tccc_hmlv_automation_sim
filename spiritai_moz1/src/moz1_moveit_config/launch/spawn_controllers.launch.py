# Spawns the joint_state_broadcaster + JTCs against the MOCK system only.
# These controllers drive simulated hardware - never the real robot.
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
