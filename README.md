# spiritai_moz1_simulation

A **self-contained simulation** of the Spirit AI **Moz1** dual-arm mobile manipulator
for **Ubuntu 22.04 / ROS 2 Humble**: the full URDF and TF tree, MoveIt 2, Nav2 +
SLAM, and a furnished pick-and-place scene — in **one colcon workspace**, with no
robot hardware, no SDK and no network bring-up.

Two simulator backends present the **identical ROS 2 contract**, which is also the
contract the real Moz1 presents, so the same `moz1_description`,
`moz1_moveit_config` and `moz1_navigation_bringup` packages drive all three:

| Backend | Packages | Compute | Use when |
|---|---|---|---|
| **Gazebo Fortress** (gz-sim) | `moz1_sim_gazebo` | laptop-friendly | **start here** — the world ships in this repo, one command to run |
| **Isaac Sim 5.1.0** | `moz1_sim_isaac` + `moz1_sim_bridge` + `moz1_sim_bringup` | RTX GPU | high-fidelity lidar/rendering, sim→real transfer, parallel RL |

**Gazebo is the default path.** Its world file is included, so the sim is runnable
straight after `colcon build`. The Isaac path ships the host-side glue and the
OmniGraph scripts but expects you to supply the Moz1 USD stage — see
[`docs/ISAAC.md`](docs/ISAAC.md).

---

## What's in here

```
src/
├── moz1_description/         URDF/xacro + CAD meshes + TF tree + home poses
├── moz1_moveit_config/       SRDF, kinematics, joint limits, planning groups, controllers
├── moz1_navigation_bringup/  tuned nav2_params.yaml, slam_toolbox, pointcloud_to_scan
├── moz1_sim_gazebo/          gz-sim worlds, launches, gz↔ROS bridge, RViz configs
├── moz1_sim_bringup/         Isaac-side launches + the topic_based_ros2_control URDF
├── moz1_sim_bridge/          Isaac joint-name remap, PointCloud2→Livox CustomMsg
└── moz1_sim_isaac/           OmniGraph scripts to paste into Isaac's Script Editor
source_sim.sh                 build + source the workspace (one overlay)
gpu_nvidia.sh                 route gz rendering to an NVIDIA dGPU (hybrid laptops)
docs/ISAAC.md                 the Isaac Sim runbook
```

The first three packages are the **same ones the real robot runs** — nothing in them
is simulation-specific. That is the point: sim↔real contract parity.

### What is deliberately *not* here

This repo is the simulation slice only. It does **not** contain the Moz1 SDK or
hardware driver, the CRP13 orchestration layer or its agents, perception
(ArUco/SAM3/VLA) or the 3D-lidar SLAM stack. Launch files that could only run on the
physical robot were removed rather than shipped broken:

- `moz1_moveit_config`: `real.launch.py` + `joint_limits_real.yaml` (need the SDK driver).
- `moz1_navigation_bringup`: the Livox-driver, FAST-LIO2, `lidarslam`/PGO,
  `lidar_localization`, `rf2o` laser-odometry and wheel-odometry-EKF launches, plus
  the configs only they used.

Everything left in the repo runs against the simulator.

---

## Install (once)

```bash
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-moveit ros-humble-xacro \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan ros-humble-topic-tools
```

`ros-humble-ros-gz` pulls in Gazebo Fortress (`ignition-gazebo6`). Gazebo Classic
(EOL Jan 2025) is **not** supported.

> **`ros-humble-ign-ros2-control` is a separate package and easy to miss** — without
> it the `controller_manager` never starts inside Gazebo and no controller activates.

For the Isaac backend, also `ros-humble-topic-based-ros2-control` — see
[`docs/ISAAC.md`](docs/ISAAC.md).

## Build & run

```bash
git clone https://github.com/artc-asr/spiritai_moz1_simulation.git
cd spiritai_moz1_simulation
source source_sim.sh build          # colcon build --symlink-install, then source
source source_sim.sh                # every new terminal
```

On a **hybrid Intel + NVIDIA laptop**, add the `gpu` flag so Gazebo's `ogre2`
renderer (and therefore the `gpu_lidar`) lands on the NVIDIA card:

```bash
source source_sim.sh gpu
glxinfo -B | grep "OpenGL renderer"   # must say NVIDIA
```

Then pick a launch. **Always `pkill -9 -f 'ign gazebo'` first** — the gz server
survives Ctrl-C and a leftover one hijacks `/clock`, after which no controller can
activate.

```bash
# robot + world + controllers only
ros2 launch moz1_sim_gazebo sim_gazebo.launch.py

# + MoveIt move_group and RViz MotionPlanning (base pinned upright for arm work)
ros2 launch moz1_sim_gazebo sim_gazebo_moveit.launch.py fix_base:=true

# + Nav2 (MPPI Omni) and slam_toolbox, free holonomic base
ros2 launch moz1_sim_gazebo sim_gazebo_nav.launch.py

# BOTH in one sim: free base + Nav2/SLAM + move_group (the full pick-and-place loop)
ros2 launch moz1_sim_gazebo sim_gazebo_full.launch.py
```

For the nav launches, also run the costmap relay in a second terminal — the shared
`nav2_params.yaml` is the real robot's, where the obstacle layer subscribes to
FAST-LIO2's `/cloud_registered_body`:

```bash
source source_sim.sh
ros2 run topic_tools relay /livox/lidar /cloud_registered_body
```

Give it ~40 s (controllers ≈25 s, then Nav2 + SLAM + RViz), then send a goal with
RViz's **2D Nav Goal** button or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

Full launch-argument table, GPU notes, RViz-only viewing, and a long
hard-won troubleshooting section:
**[`src/moz1_sim_gazebo/README.md`](src/moz1_sim_gazebo/README.md)** — read it before
filing a bug.

---

## The ROS 2 contract

Identical across Gazebo, Isaac and the real robot. Bare topics, no namespace,
everything on `use_sim_time:=true`.

| Direction | Topic / interface | Type | Provided by (Gazebo) |
|---|---|---|---|
| PUB | `/livox/lidar` (frame `livox_frame`) | `sensor_msgs/PointCloud2` | gz `gpu_lidar` → `ros_gz_bridge` |
| PUB | `/livox/imu` | `sensor_msgs/Imu` | gz `imu` sensor → bridge |
| PUB | `/odom` + TF `odom→base_link` | `nav_msgs/Odometry` | gz `OdometryPublisher` → bridge |
| PUB | `/joint_states` | `sensor_msgs/JointState` | `ign_ros2_control` + `joint_state_broadcaster` |
| PUB | `/cam_high/image`, `/cam_{left,right}_wrist/image` (+ `depth_image`, `camera_info`) | `sensor_msgs/Image`, `CameraInfo` | gz camera sensors → bridge |
| SUB | `/cmd_vel` (vx, vy, wz — holonomic) | `geometry_msgs/Twist` | bridge → gz `VelocityControl` |
| both | `FollowJointTrajectory` on `left_arm`, `right_arm`, `torso`, `left_gripper`, `right_gripper` | `control_msgs/action` | `ign_ros2_control` (native ROS 2, no bridge) |
| SUB | `/grasp/{left,right}/{attach,detach}` | `std_msgs/Empty` | sim-only grasp weld (`scripts/grasp_helper.py`) |

MoveIt planning groups (from `moz1.srdf`): `left_arm` (7 DoF), `right_arm` (7 DoF),
`dual_arm`, `torso` (6 DoF), `left_gripper`, `right_gripper`, with `home` / `zero` /
`open` / `closed` named states and `left_ee` / `right_ee` end effectors.

> The SRDF pins `base_link` to `world` with a **fixed** virtual joint — the mobile
> base is out of MoveIt. `sim_gazebo_moveit.launch.py` therefore defaults
> `fix_base:=true`; `sim_gazebo_full.launch.py` keeps the base free and publishes a
> static `world→map` instead, relying on the base parking before the arm plans. The
> reasoning and its caveats are in that launch file's docstring.

## The scene

`moz1_pickplace.world` (the default) is a ~9 × 8 m walled room, fully inline SDF —
no external model URIs, nothing to download:

| Fixture | Where | For |
|---|---|---|
| **workbench** + two bins | `x ≈ 1.8`, front | the **bearing ring** pick-and-place task |
| **4-tier shelf** + trays | `x ≈ −1.9`, behind | the **sample bottle** task (reach at 1.06 m) |
| **place table** | `y = 2.0` | drop-off station |
| desk, monitor, chair, cart | scattered | clutter, costmap obstacles, nav landmarks |
| 4 walls | −3…6 × −4…4 | bounded map for SLAM |

`bearing_ring` and `sample_bottle` are the graspable objects (the rest are static);
the `*_filler_*` bodies are distractors. `moz1_lab.world` is the bare alternative —
ground plane only — for pure locomotion or controller work.

Running Gazebo headless? `scripts/scene_markers.py` (started automatically by the
nav launches) republishes the world's fixtures as RViz markers on `/scene_markers`,
so you can work entirely in RViz.

---

## Known limitations (verified on this workspace)

Smoke-tested headless on an NVIDIA RTX PRO 6000, ROS 2 Humble, Gazebo Fortress.
**Working:** sim steps and publishes `/clock`; `/joint_states` reports all 24 joints;
`joint_state_broadcaster`, `torso_controller`, `left_arm_controller` and
`right_arm_controller` activate; arm `FollowJointTrajectory` goals execute to target
(`error_code: SUCCESSFUL`); `/cmd_vel` drives the holonomic base (forward + strafe
confirmed against `/odom`); MoveIt `move_group` comes up on the Gazebo robot and a
`right_arm` joint-space goal plans with OMPL and **executes** (21-point trajectory,
`error_code: SUCCESS`, joints land inside tolerance); all 22 launch files introspect
cleanly.

**Not working — the grippers do not activate.** `left_gripper_controller` and
`right_gripper_controller` load and configure but fail to activate:

```
resource_manager: Not acceptable command interfaces combination:
  Not existing: [ left_gripper_joint/position ]
gz_ros2_control: Skipping joint in the URDF named 'left_gripper_joint'
                 which is not in the gazebo model.
urdf2sdf: parent joint[left_gripper_joint] ignored.
```

Root cause: `{left,right}_gripper_joint`'s child link (`*_gripper_actuator_link`) is a
pure frame — no visual, no collision, no inertial. `sim_gazebo.launch.py`'s
`_inject_default_inertials` only injects into links that *have* geometry, so this one
stays inertial-less; `urdf2sdf` then drops the link **and the prismatic joint above
it**, and `gz_ros2_control` has no command interface to claim. The same lumping breaks
the sim-only grasp weld (`DetachableJoint: Link with name [left_gripper_base_link] not
found in model [moz1]`).

This is **pre-existing upstream**, not an artifact of this extraction —
`sim_gazebo.launch.py`, `moz1_gazebo.urdf.xacro` and all of `moz1_description` are
byte-identical to the internal repo. Arm planning and execution, navigation and the
scene are unaffected; only closing a gripper on an object in Gazebo is. The likely fix
is to widen `_inject_default_inertials` to cover geometry-less links that are the child
of a non-fixed joint, and to point the `DetachableJoint` `parent_link` at the
post-lumping link name.

---

## Provenance

These packages are extracted from Spirit AI's internal `spirit_ai_dev` and CRP13
workspaces and flattened into a single workspace for simulation-only use. Upstream,
`moz1_description` / `moz1_moveit_config` live in `moz1_manip_ws`,
`moz1_navigation_bringup` in `moz1_nav_ws`, and the `moz1_sim_*` packages in
`moz1_sim_ws`, sourced as three ordered overlays. Here they are one build — but the
packages themselves are unchanged, so fixes can move either way.

`moz1_description`'s torso and arm geometry comes from official Spirit AI CAD
(`spirit01_model`). Grippers, EE tips and sensor frames are clearly-marked
placeholders. Link inertials are placeholder mass/inertia added for Gazebo physics —
they are ignored by RViz, MoveIt and the real robot.
