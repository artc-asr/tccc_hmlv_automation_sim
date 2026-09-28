"""
attach_sensors.py  --  Moz1 Isaac Sim sensor + odometry interface (Phase 2).

Paste into Isaac Sim 5.1.0 >> Window > Script Editor and Run (then press PLAY).
Builds an OmniGraph + RTX Lidar that make the sim publish the SAME nav-relevant
topics the real robot does, so the unchanged moz1_navigation_bringup stack
(pointcloud_to_laserscan + slam_toolbox + ekf, or FAST-LIO2) consumes them:

  PUB /livox/lidar  sensor_msgs/PointCloud2   frame livox_frame   (RTX Mid-360)
  PUB /livox/imu    sensor_msgs/Imu           frame livox_frame   (~200 Hz)
  PUB /odom         nav_msgs/Odometry         odom -> base_link   (ground truth)
  PUB /tf           tf2_msgs/TFMessage        odom -> base_link   (raw TF)

The static base_link -> livox_frame TF is published HOST-SIDE by the bringup
launch (mirrors livox_bringup.launch.py), so the lidar prim is created at the
configured mount offset and its ROS frameId is set to 'livox_frame'.

Re-runnable: tears down a previous /World/Moz1SimInterface first.
ROS_DOMAIN_ID must be 33 (match setup_env.sh) and the Isaac ROS2 bridge enabled.
"""

import math

import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, Sdf, UsdGeom

# ------------------------------------------------------------------ #
# CONFIG  (keep in sync with interface_config.py)
# ------------------------------------------------------------------ #
ROBOT_PRIM_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/base_link"
LIDAR_PARENT_PRIM_PATH = ROBOT_PRIM_PATH
CAMERA_PRIM_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/head23/camera0_link/head_camera"

TOPIC_ODOM = "/odom"
TOPIC_LIVOX_LIDAR = "/livox/lidar"
TOPIC_LIVOX_IMU = "/livox/imu"

FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_LIVOX = "livox_frame"

LIVOX_MOUNT_XYZ = (0.0, 0.0, 0.4)   # base_link -> livox; match livox_bringup tf_z
# RTX Lidar config: ship config/mid360.json with this package and pass an
# absolute path, OR use a built-in Isaac config name if your install has one.
# Set to an absolute path string (recommended) or a built-in config name.
LIDAR_CONFIG = "Mid360"             # TODO: swap to "/abs/path/to/mid360.json" if needed

GRAPH_PATH = "/World/Moz1SimInterface"
SENSOR_GRAPH = GRAPH_PATH + "/Sensors"

# ------------------------------------------------------------------ #
# Teardown previous attachment (safe re-run)
# ------------------------------------------------------------------ #
stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No stage loaded. Open the Spirit Moz1 scene first.")
if not stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
    raise RuntimeError(f"Robot prim not found: {ROBOT_PRIM_PATH} (run discover_prims.py)")

for path in (SENSOR_GRAPH, GRAPH_PATH + "/Lidar", GRAPH_PATH + "/Imu"):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
        print(f"[sensors] Removed previous {path}")

# ------------------------------------------------------------------ #
# 1) RTX Lidar (Livox Mid-360 approximation) under base_link
# ------------------------------------------------------------------ #
lidar_path = GRAPH_PATH + "/Lidar/livox_mid360"
_, lidar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_path,
    parent=None,
    config=LIDAR_CONFIG,
    translation=Gf.Vec3d(*LIVOX_MOUNT_XYZ),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
# Parent the lidar to base_link so it rides with the robot.
omni.kit.commands.execute(
    "MovePrim", path_from=lidar_path,
    path_to=LIDAR_PARENT_PRIM_PATH + "/livox_mid360",
)
lidar_path = LIDAR_PARENT_PRIM_PATH + "/livox_mid360"
lidar_rp = rep.create.render_product(lidar_path, resolution=(1, 1))
print(f"[sensors] RTX Lidar at {lidar_path}  rp={lidar_rp.path}")

# ------------------------------------------------------------------ #
# 2) OmniGraph: lidar->PointCloud2, IMU, odometry, raw TF
# ------------------------------------------------------------------ #
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": SENSOR_GRAPH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),

            # Lidar point cloud publisher (sensor_msgs/PointCloud2).
            ("LidarPC", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),

            # IMU: read the articulation IMU and publish sensor_msgs/Imu.
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PubIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),

            # Odometry + raw TF (odom -> base_link).
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PubRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        keys.SET_VALUES: [
            # PointCloud2 from the RTX lidar render product, frame livox_frame.
            ("LidarPC.inputs:renderProductPath", lidar_rp.path),
            ("LidarPC.inputs:topicName", TOPIC_LIVOX_LIDAR),
            ("LidarPC.inputs:frameId", FRAME_LIVOX),
            ("LidarPC.inputs:type", "point_cloud"),
            ("LidarPC.inputs:fullScan", True),

            # IMU sensor lives on the lidar prim so /livox/imu shares its frame.
            ("ReadIMU.inputs:imuPrim", [Sdf.Path(lidar_path)]),
            ("PubIMU.inputs:topicName", TOPIC_LIVOX_IMU),
            ("PubIMU.inputs:frameId", FRAME_LIVOX),

            # Odometry from the chassis articulation.
            ("ComputeOdom.inputs:chassisPrim", [Sdf.Path(ROBOT_PRIM_PATH)]),
            ("PubOdom.inputs:topicName", TOPIC_ODOM),
            ("PubOdom.inputs:odomFrameId", FRAME_ODOM),
            ("PubOdom.inputs:chassisFrameId", FRAME_BASE),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "LidarPC.inputs:execIn"),
            ("OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),

            ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubRawTF.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubIMU.inputs:timeStamp"),

            # IMU read -> publish.
            ("ReadIMU.outputs:execOut", "PubIMU.inputs:execIn"),
            ("ReadIMU.outputs:angVel", "PubIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PubIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PubIMU.inputs:orientation"),

            # Odometry compute -> odom publisher.
            ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),

            # Odometry compute -> raw TF (odom -> base_link).
            ("ComputeOdom.outputs:execOut", "PubRawTF.inputs:execIn"),
            ("ComputeOdom.outputs:orientation", "PubRawTF.inputs:rotation"),
            ("ComputeOdom.outputs:position", "PubRawTF.inputs:translation"),
        ],
    },
)

print("[sensors] Attached at", SENSOR_GRAPH)
print("  PUB", TOPIC_LIVOX_LIDAR, " sensor_msgs/PointCloud2  frame", FRAME_LIVOX)
print("  PUB", TOPIC_LIVOX_IMU, "   sensor_msgs/Imu          frame", FRAME_LIVOX)
print("  PUB", TOPIC_ODOM, "         nav_msgs/Odometry        ", FRAME_ODOM, "->", FRAME_BASE)
print("  PUB /tf                 odom -> base_link (raw)")
print("")
print("NOTE: static base_link->livox_frame TF is published host-side by")
print("      moz1_sim_bringup (mirrors livox_bringup.launch.py). Press PLAY.")
