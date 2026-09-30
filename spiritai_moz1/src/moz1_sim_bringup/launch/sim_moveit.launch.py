"""
sim_moveit.launch.py  --  MoveIt against the Moz1 Isaac Sim arm.

Brings up the SAME MoveIt stack as moz1_moveit_config/demo.launch.py (move_group,
RViz MotionPlanning, robot_state_publisher, controller_manager + the four
JointTrajectoryControllers, static world->base_footprint TF) but backs
ros2_control with topic_based_ros2_control instead of mock_components, so the
controllers exchange state/commands with Isaac Sim over /joint_states and
/joint_command (canonical names via joint_name_remap).

Everything runs on use_sim_time so MoveIt plans from Isaac's clock. SRDF,
kinematics, joint limits and the controllers list are reused byte-identical from
moz1_moveit_config; only the <ros2_control> hardware block differs (our
config/moz1_sim.urdf.xacro).

Prereq: run sim_bridge.launch.py (joint_name_remap) and the Isaac-side
attach_joint_io.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    sim_xacro = os.path.join(
        get_package_share_directory("moz1_sim_bringup"),
        "config", "moz1_sim.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("moz1", package_name="moz1_moveit_config")
        # Swap the description for the topic_based_ros2_control variant. SRDF,
        # kinematics, controllers, joint_limits still come from moz1_moveit_config.
        .robot_description(file_path=sim_xacro)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # generate_demo_launch wires move_group, rviz, rsp, ros2_control_node
    # (reading the topic_based hardware from robot_description) and the spawners.
    demo = generate_demo_launch(moveit_config)

    # Force every node (including the included ones) onto the sim clock.
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        *demo.entities,
    ])
