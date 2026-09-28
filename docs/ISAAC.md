# Isaac Sim backend — drive the Moz1 stack against Isaac Sim 5.1.0

The Isaac path makes the **Spirit AI Moz1 Isaac Sim 5.1.0** scene present the same
ROS 2 contract as the Gazebo backend and the real robot, so the **unchanged**
`moz1_description` / `moz1_navigation_bringup` / `moz1_moveit_config` packages in
this workspace drive it.

For the lighter, fully self-contained option (world file included, no GPU
requirement beyond lidar rendering), use the **Gazebo** backend instead —
[`src/moz1_sim_gazebo/README.md`](../src/moz1_sim_gazebo/README.md). Start there
unless you specifically need Isaac's fidelity or its parallelism.

> **Isaac needs a scene you supply.** Unlike Gazebo (whose world ships in
> `moz1_sim_gazebo/worlds/`), the Isaac path expects the Moz1 USD stage to already
> be open in Isaac Sim. This repo ships the **host-side glue and the OmniGraph
> scripts**, not the USD.

> **Core principle:** we do *not* re-implement Nav2/SLAM/MoveIt. The only new code
> is a thin **"sim robot replacement" layer** that emulates the topics the robot's
> embedded controller (`172.16.0.20` — the real DDS publisher; MovaX is a TCP
> operator console, not a ROS node) and `livox_ros_driver2` publish. The sim
> replaces the controller's low-level graph and the Livox driver — never the
> planners.

## The contract

Bare topics, no namespace, `ROS_DOMAIN_ID=33`, `use_sim_time:=true`.

Sim **publishes** (sim → stack):

| Topic | Type | Frame | Produced by |
|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `livox_frame` | `attach_sensors.py` (RTX Mid-360) |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_frame` | `attach_sensors.py` |
| `/odom` + TF `odom→base_link` | `nav_msgs/Odometry` | `odom`/`base_link` | `attach_sensors.py` (ground truth) |
| `/isaac/joint_states` → `/joint_states` | `sensor_msgs/JointState` | — | `attach_joint_io.py` → `joint_name_remap` |

Sim **subscribes** (stack → sim):

| Topic | Type | Consumed by |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `attach_base_control.py` (mecanum IK) |
| `/joint_command` → `/isaac/joint_command` | `sensor_msgs/JointState` | `joint_name_remap` → `attach_joint_io.py` |

Static `base_link→livox_frame` (z≈0.4) is published host-side by
`sim_bridge.launch.py` (mirroring the real `livox_bringup.launch.py`, since the sim
runs no real Livox driver).

## Packages

- **`moz1_sim_isaac`** — paste-into-Isaac-Script-Editor OmniGraph scripts
  (`isaac_scripts/`). These run inside the Isaac Sim Python (`omni.*`), **not** on
  the host, so nothing here imports them at build time.
  - `discover_prims.py` — Phase 1: enumerate every joint + DOF + lidar/camera prims,
    and print a **remap skeleton** for `joint_name_map.yaml`.
  - `attach_sensors.py` — RTX Livox Mid-360 + IMU + odometry + raw TF.
  - `attach_base_control.py` — `/cmd_vel` → 4 mecanum wheel velocities.
  - `attach_joint_io.py` — `/isaac/joint_states` + `/isaac/joint_command` ↔ articulation.
  - `config/mid360.json` — RTX lidar config approximating the Mid-360.
- **`moz1_sim_bridge`** — host ROS 2 (Humble) nodes.
  - `joint_name_remap` — sim joint names ↔ canonical URDF names, both directions.
  - `pc2_to_livox` — optional `PointCloud2 → livox_ros_driver2/CustomMsg` for FAST-LIO2.
  - `config/joint_name_map.yaml` — fill from `discover_prims.py` output.
- **`moz1_sim_bringup`** — launch orchestration on `use_sim_time`.
  - `sim_bridge.launch.py`, `sim_nav.launch.py`, `sim_moveit.launch.py`,
    `sim_full_stack.launch.py`, and `config/moz1_sim.urdf.xacro` (the
    `topic_based_ros2_control` hardware variant of the MoveIt description).

## Prerequisites

Beyond the [common install](../README.md#install-once):

```bash
sudo apt install ros-humble-topic-based-ros2-control
```

`topic_based_ros2_control` is what lets `moz1_moveit_config`'s controllers exchange
state and commands with Isaac over topics instead of looping internally.

**`livox_ros_driver2` / `ws_livox`** — only needed for the FAST-LIO2 path
(`pc2_to_livox`), and **not shipped here**. Build it from
[Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
(ROS 2 branch) into its own overlay and source that before using `pc2_to_livox`.
The `slam_toolbox` path needs none of it.

Isaac Sim 5.1.0 with the `isaacsim.ros2.bridge` extension enabled, on
`ROS_DOMAIN_ID=33`, DDS reachable from the host (`source source_sim.sh isaac` sets
the host side).

### Obtaining Isaac Sim 5.1.0

Isaac Sim 5.1.0 is distributed by NVIDIA three ways — pick one:

| Method | Get it | Notes |
|---|---|---|
| **Workstation build** (recommended for the GUI Script Editor) | [`isaac-sim-standalone-5.1.0-linux-x86_64.zip`](https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip) (or [`-linux-aarch64.zip`](https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-5.1.0-linux-aarch64.zip)) | Unzip, then run `./post_install.sh` and `./isaac-sim.selector.sh`. Steps: [Quick Install docs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/quick-install.html). |
| **Container** (headless / servers) | `docker pull nvcr.io/nvidia/isaac-sim:5.1.0` from the [NGC catalog](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim) (needs a free NGC account + API key). Official [Dockerfiles](https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles). | Run with host networking (or a shared bridge) so DDS on `ROS_DOMAIN_ID=33` reaches the host. |
| **Source** | [github.com/isaac-sim/IsaacSim/releases](https://github.com/isaac-sim/IsaacSim/releases) | Isaac Sim is open source as of 5.x; build from the tagged `v5.1.0` release. |

> The official Linux **workstation** download is a `.zip`, not a `.tar`. If you were
> handed an `isaac-sim-5.1.0.tar`, it is almost certainly a `docker save` of the NGC
> container above — load it with `docker load -i isaac-sim-5.1.0.tar`.

## Bring-up sequence

**Isaac side** (Window → Script Editor, paste each file; press **Play**):

1. `attach_sensors.py` — verify prim paths first with `discover_prims.py`.
2. `attach_base_control.py`
3. `attach_joint_io.py`

**Phase 1 (once):** run `discover_prims.py`, copy its REMAP SKELETON into
`src/moz1_sim_bridge/config/joint_name_map.yaml`, and hand-correct the guesses.
Confirm the movable joint count is 26 (4 wheels + 6 torso + 7 + 7 arms + 2 grippers)
for the full dual-arm Moz1; if arms are absent, the MoveIt half is scoped down.

**Host side:**

```bash
source source_sim.sh isaac
# Everything:
ros2 launch moz1_sim_bringup sim_full_stack.launch.py
# Or one half at a time:
ros2 launch moz1_sim_bringup sim_nav.launch.py
ros2 launch moz1_sim_bringup sim_moveit.launch.py
# FAST-LIO2 path instead of slam_toolbox: add fastlio:=true and point the
# FAST-LIO2 config's lid_topic at /livox/lidar_custom.
```

## Verify (end-to-end)

1. `ros2 topic list` shows the bare contract; `ros2 topic hz /livox/lidar` ≈10 Hz,
   `/livox/imu` ≈200 Hz, `/joint_states` ≥10 Hz.
2. `ros2 run tf2_tools view_frames` → `map→odom→base_link→livox_frame` + arm chain.
3. **Nav2:** send a goal (RViz *2D Nav Goal* or `/navigate_to_pose`) → costmaps fill
   from the sim lidar → the base drives to the goal in Isaac (holonomic, incl. strafe).
4. **MoveIt:** plan + execute an arm goal in RViz → the sim arm tracks →
   `/joint_states` updates.
5. **Fidelity:** confirm `slam_toolbox` (and/or FAST-LIO2) builds a coherent map
   from the simulated Livox stream.

> Nav2's obstacle layer in the shared `nav2_params.yaml` subscribes to FAST-LIO2's
> `/cloud_registered_body`. On the `slam_toolbox` path, relay the raw cloud:
> `ros2 run topic_tools relay /livox/lidar /cloud_registered_body`.

## Known approximations / tuning knobs

- **Mid-360 fidelity:** `mid360.json` is a dense rotary approximation of the real
  non-repetitive scan; tune density/range against real `/livox/lidar` before
  trusting sim→real map transfer. The FAST-LIO2 path needs `pc2_to_livox` (Isaac
  can't emit `CustomMsg`); the `slam_toolbox` path does not.
- **Joint names:** the sim USD uses `Base_0` (underscore) vs real `Base-0`; arm names
  are confirmed in Phase 1 and mapped in `joint_name_map.yaml`.
- **IMU units:** match Isaac's IMU output to what the consumer expects (the real
  `ekf.yaml` — not shipped here — notes accel in g vs m/s²).
- **Mecanum IK** signs/geometry in `attach_base_control.py` are tuned to the sim USD;
  re-verify after any USD change.
- **Odometry:** sim uses Isaac ground-truth `odom→base_link`; `slam_toolbox` supplies
  `map→odom`. The real EKF (`odom_ekf`) is intentionally not run in sim, and is not
  part of this repo.
