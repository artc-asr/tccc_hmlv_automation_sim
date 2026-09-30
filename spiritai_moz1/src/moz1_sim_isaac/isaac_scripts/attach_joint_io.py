"""
attach_joint_io.py  --  Moz1 Isaac Sim joint state + command interface (Phase 3).

Paste into Isaac Sim 5.1.0 >> Window > Script Editor and Run (then PLAY).
Bridges the arm/torso/gripper articulation to ROS2 so MoveIt (via
topic_based_ros2_control on the host) can read state and command trajectories:

  PUB /joint_states   sensor_msgs/JointState   (SIM joint names, all DOF)
  SUB /joint_command  sensor_msgs/JointState   (SIM joint names -> articulation)

The host-side moz1_sim_bridge/joint_name_remap relays canonical URDF names
(LeftArm-0..6, ...) <-> the sim joint names on these two topics, so MoveIt's
controllers stay byte-identical to the real moz1_moveit_config.

Re-runnable: tears down /World/Moz1SimInterface/JointIO first.
"""

import omni.graph.core as og
import omni.usd
from pxr import Sdf

# ------------------------------------------------------------------ #
# CONFIG (keep in sync with interface_config.py)
# ------------------------------------------------------------------ #
ROBOT_PRIM_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/base_link"
# Articulation ROOT to publish/command. Often the same as ROBOT_PRIM_PATH; if
# discover_prims.py shows the ArticulationRootAPI on a different prim, use that.
ARTICULATION_ROOT = ROBOT_PRIM_PATH

# RAW sim-side topics (carry SIM joint names). The host-side joint_name_remap
# bridges these to canonical /joint_states + /joint_command. If discover_prims
# shows the sim already uses canonical names, point these straight at
# /joint_states + /joint_command and skip the remap node.
TOPIC_JOINT_STATES = "/isaac/joint_states"
TOPIC_JOINT_COMMAND = "/isaac/joint_command"

GRAPH_PATH = "/World/Moz1SimInterface"
JOINT_GRAPH = GRAPH_PATH + "/JointIO"

# ------------------------------------------------------------------ #
# Teardown previous attachment
# ------------------------------------------------------------------ #
stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No stage loaded. Open the Spirit Moz1 scene first.")
if not stage.GetPrimAtPath(ARTICULATION_ROOT).IsValid():
    raise RuntimeError(f"Articulation root not found: {ARTICULATION_ROOT} "
                       f"(run discover_prims.py)")
if stage.GetPrimAtPath(JOINT_GRAPH).IsValid():
    stage.RemovePrim(JOINT_GRAPH)
    print(f"[jointio] Removed previous {JOINT_GRAPH}")

# ------------------------------------------------------------------ #
# OmniGraph: publish joint states, subscribe joint commands -> articulation
# ------------------------------------------------------------------ #
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": JOINT_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),

            ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubJoints", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("PubJoints.inputs:targetPrim", [Sdf.Path(ARTICULATION_ROOT)]),
            ("PubJoints.inputs:topicName", TOPIC_JOINT_STATES),

            ("SubJoints.inputs:topicName", TOPIC_JOINT_COMMAND),

            ("ArtController.inputs:targetPrim", [Sdf.Path(ARTICULATION_ROOT)]),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "PubJoints.inputs:execIn"),
            ("OnTick.outputs:tick", "SubJoints.inputs:execIn"),
            ("OnTick.outputs:tick", "ArtController.inputs:execIn"),

            ("SimTime.outputs:simulationTime", "PubJoints.inputs:timeStamp"),

            # Commanded JointState -> articulation drive targets, matched by name.
            ("SubJoints.outputs:jointNames", "ArtController.inputs:jointNames"),
            ("SubJoints.outputs:positionCommand", "ArtController.inputs:positionCommand"),
            ("SubJoints.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
            ("SubJoints.outputs:effortCommand", "ArtController.inputs:effortCommand"),
        ],
    },
)

print("[jointio] Attached at", JOINT_GRAPH)
print("  PUB", TOPIC_JOINT_STATES, " sensor_msgs/JointState (sim names)")
print("  SUB", TOPIC_JOINT_COMMAND, "sensor_msgs/JointState (sim names) -> articulation")
print("")
print("Host side: run moz1_sim_bridge joint_name_remap to map canonical <-> sim")
print("names, then MoveIt's topic_based_ros2_control drives /joint_command.")
