# Spirit AI Moz1 — Gazebo Fortress / Isaac Sim

Part of [tccc_hmlv_automation_sim](../README.md). Everything below is relative to
this directory (`spiritai_moz1/`), which is its own colcon workspace.

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
../gpu_nvidia.sh              route gz rendering to an NVIDIA dGPU (shared, at the repo root)
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

> **Got `ros-humble-ros-gzharmonic` instead?** It conflicts with `ros-humble-ros-gz`,
> and its bridge speaks gz-transport13, which never sees Fortress topics (`/clock`,
> the grasp topics … all silent). Build `ros_gz` (branch `humble`) for Fortress in a
> workspace beside the repo checkout — `source_sim.sh` overlays
> `<checkout>/../gz_fortress_ws` automatically
> (override with `MOZ1_GZ_WS`):
>
> ```bash
> mkdir -p ../../gz_fortress_ws/src && cd ../../gz_fortress_ws/src   # from spiritai_moz1/
> git clone -b humble https://github.com/gazebosim/ros_gz.git
> git clone -b ros2-devel https://github.com/swri-robotics/gps_umd.git   # gps_msgs
> cd .. && source /opt/ros/humble/setup.bash && export GZ_VERSION=fortress
> colcon build --packages-select gps_msgs ros_gz_interfaces ros_gz_bridge ros_gz_sim \
>   --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
> ```
>
> `ros-humble-gz-ros2-control` is enough for the controllers: it ships the
> `ign_ros2_control` plugin names too, and the launch falls back to it.

## Build & run

```bash
git clone https://github.com/artc-asr/tccc_hmlv_automation_sim.git
cd tccc_hmlv_automation_sim/spiritai_moz1
source source_sim.sh build          # colcon build --symlink-install, then source
source source_sim.sh                # every new terminal
```

On a **hybrid Intel + NVIDIA laptop**, add the `gpu` flag so Gazebo's `ogre2`
renderer (and therefore the `gpu_lidar`) lands on the NVIDIA card:

```bash
source source_sim.sh gpu
glxinfo -B | grep "OpenGL renderer"   # must say NVIDIA
```

Then pick a launch. **Always `../sim.sh stop` (or `pkill -9 -f 'ign gazebo'`) first** — the gz server
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

# jerry-can depalletising demo: pallet of 4 L cans → both arms → infeed conveyors
# (Gazebo headless, RViz shows everything; add gui:=true for the gz window)
ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py

```

The **jerry-can transfer** demo (both arms pick an empty pair, conveyor + filling to
4.2 kg, drive, place the filled pair in a box on a second pallet) moved to the shared
cell, [`../hmlv_cell/`](../hmlv_cell/README.md), where the Galbot G1 can run it too. It
overlays this workspace and still uses this package's plugins:

```bash
cd ../hmlv_cell && source source_sim.sh build
ros2 launch hmlv_cell_gazebo transfer.launch.py            # robot:=moz1 is the default
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
| SUB | `/grasp/{left,right}/{attach,detach}`, `/grasp/{left,right}/target` | `std_msgs/Empty`, `std_msgs/String` | sim-only grasp weld (`scripts/grasp_helper.py`): welds the selected target on gripper close |

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

### Jerry-can depalletising cell

`sim_gazebo_jerrycan.launch.py` generates a second scene (`scripts/jerrycan_scene.py`):
the HMLV liquid-filling-line cell from the humanoid pallet-reach check — a EUR pallet
(1200 × 1000 × 150 mm, worked from the 1200 side, 50 mm from the base) stacked with
TCCC HDPE jerry cans, and an infeed conveyor (belt at 0.75 m) on each side of the
robot. The base is pinned; `scripts/jerrycan_demo.py` then moves the top-front row
onto the conveyors, alternating arms, through move_group (OMPL transit + Cartesian
approach/lift/place):

| `container:=` | can (W × L × h) | filled @ 1.0 g/mL | stack | targets | vs. 5 kg/arm placeholder | single-arm result in the sim |
|---|---|---|---|---|---|---|
| `4` (default) | 120 × 180 × 240 mm | 4.20 kg | 3 layers × 8 rows × 6 | 6 | within | **6/6** placed, 0–10 mm off the slot (two consecutive runs) |
| `10` | 220 × 270 × 240 mm | 10.55 kg | 3 × 4 × 4 | 4 | **over** | 0/4 — the arm sags 16 cm on grasping: ~52 N·m needed at the shoulder vs. the URDF's 50 N·m limit |
| `20` | 215 × 270 × 420 mm | 21.05 kg | 2 × 4 × 4 | 4 | **over** | not run; twice the 10 L load |

Every place is checked against Gazebo's own model poses, so the final
`done: N/M jerry cans verified on the conveyors` counts cans that are really standing
on a slot. The held can's weight acts on the arm, so payload limits show up: 10 L
fails single-armed exactly as the reach check predicts (it needs both arms, which
the demo doesn't do yet). The Moz1's real per-arm payload and joint torques are
unpublished — the URDF's 50 N·m arm effort limit is what the sim enforces.
Arguments and design notes: [`src/moz1_sim_gazebo/README.md`](src/moz1_sim_gazebo/README.md#jerry-can-depalletising-demo).

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

**Grippers — fixed.** Upstream, `left_gripper_controller` / `right_gripper_controller`
never activated (`Not existing: [ left_gripper_joint/position ]`): the prismatic
`*_gripper_joint`'s child `*_gripper_actuator_link` is a pure frame, so urdf2sdf dropped
the link and the joint, and the lumped-away `*_gripper_base_link` broke the grasp
weld (`DetachableJoint: Link with name [left_gripper_base_link] not found`). Now:

- `_inject_default_inertials` also gives a tiny inertial to geometry-less links under
  a **movable** joint, so the gripper joint survives;
- `moz1_gazebo.urdf.xacro` preserves the gripper mount joint (`preserveFixedJoint`), so
  `*_gripper_base_link` exists in gz, and drives the 6 finger joints per hand as
  `gz_ros2_control` mimic joints (no offset term in Humble: fingers sit a few degrees
  off the URDF pose — visual only);
- the grasp plugins are generated per `(side, object)` from the `grasp_targets`
  launch argument, each on its own topic pair. They are `KinematicGrasp`
  (`src/kinematic_grasp.cc`), a rigid drop-in for gz's DetachableJoint, whose
  soft two-model weld let heavy objects swing in the hand.

All six controllers activate; closing a gripper welds its selected object.

**Fixed base spawns at z = 0.01** (was 0.12, i.e. hovering): base_link lands at 0.11
with the r = 0.105 m wheels 5 mm clear. Spawning at 0 sinks them 5 mm into the ground
and the sim drops to ~1 step / 10 s — the same collapse the free base had at z = 0.05.

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
