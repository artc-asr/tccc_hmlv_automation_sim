"""
interface_config.py  --  shared config for the Moz1 Isaac Sim attach scripts.

These values are duplicated as plain constants at the top of each attach_*.py
script so they can be pasted standalone into the Isaac Script Editor. Keep this
file as the single source of truth and re-sync the headers if you change a path.

IMPORTANT: topic names here intentionally match the REAL robot's bare-topic
contract (no /spirit namespace) so the unchanged Nav2/SLAM/MoveIt stack drives
the sim exactly as it drives hardware. ROS_DOMAIN_ID must be 33 on both sides.
"""


class Moz1SimConfig:
    # ---- Stage prim paths (verify with discover_prims.py; override if different) ----
    # The articulation root / chassis link of the Moz1 in the Spirit AI scene.
    ROBOT_PRIM_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/base_link"
    # Scope that holds the wheel/arm joints (the '/joints' sibling under MOZ1).
    JOINTS_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/joints"
    # Head camera prim (RGB-D, optional for nav but kept for parity / training).
    CAMERA_PRIM_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/head23/camera0_link/head_camera"
    # Link the simulated Livox is rigidly attached to (its frame == livox_frame).
    LIDAR_PARENT_PRIM_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/base_link"

    # ---- ROS2 topics: MATCH THE REAL ROBOT (bare, no namespace) ----
    TOPIC_CMD_VEL = "/cmd_vel"                # SUB  geometry_msgs/Twist  (Nav2 -> sim)
    TOPIC_ODOM = "/odom"                      # PUB  nav_msgs/Odometry    (sim -> stack)
    TOPIC_LIVOX_LIDAR = "/livox/lidar"        # PUB  sensor_msgs/PointCloud2
    TOPIC_LIVOX_IMU = "/livox/imu"            # PUB  sensor_msgs/Imu
    # RAW sim-side joint topics (carry SIM joint names); host joint_name_remap
    # bridges them to canonical /joint_states + /joint_command for MoveIt.
    TOPIC_JOINT_STATES = "/isaac/joint_states"    # PUB  sensor_msgs/JointState (sim names)
    TOPIC_JOINT_COMMAND = "/isaac/joint_command"  # SUB  sensor_msgs/JointState (sim names)
    TOPIC_CAMERA_RGB = "/camera/image_raw"
    TOPIC_CAMERA_DEPTH = "/camera/depth"
    TOPIC_CAMERA_INFO = "/camera/camera_info"

    # ---- Frames (match real stack) ----
    FRAME_ODOM = "odom"
    FRAME_BASE = "base_link"
    FRAME_LIVOX = "livox_frame"               # static base_link->livox_frame, z~0.4
    FRAME_CAMERA = "camera_link"

    # ---- Livox Mid-360 mounting (matches livox_bringup.launch.py default) ----
    LIVOX_MOUNT_XYZ = (0.0, 0.0, 0.4)
    LIVOX_MOUNT_RPY = (0.0, 0.0, 0.0)

    # ---- Mecanum X-drive geometry (re-verify against the sim USD) ----
    WHEEL_RADIUS = 0.08
    WHEEL_RADIAL_DIST = 0.342644              # sqrt(2) * 0.2423

    # ---- OmniGraph root (everything we add lives here; easy to detach) ----
    GRAPH_PATH = "/World/Moz1SimInterface"
