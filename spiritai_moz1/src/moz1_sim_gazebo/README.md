# moz1_sim_gazebo — Moz1 in Gazebo Fortress (gz-sim)

A **Gazebo Fortress** (new "Gazebo Sim", via `ros_gz`) simulation of the Moz1 for
**Ubuntu 22.04 / ROS 2 Humble**. It presents the **same ROS2 contract** as the
real robot and the Isaac sim (`moz1_sim_ws`), so the unchanged
`moz1_navigation_bringup` / `moz1_moveit_config` drive it unchanged —
a lighter-compute alternative to Isaac for **manipulation / control** work.
Gazebo Classic (EOL Jan 2025) is **not** supported — Fortress only.

> **Verified working end-to-end** — base + both arms + grippers + odom + imu +
> **`gpu_lidar`** — on an NVIDIA GPU. The `gpu_lidar` (and thus SLAM/Nav2) needs a
> working render engine (ogre2); on a hybrid Intel+NVIDIA laptop you route gz to
> the NVIDIA card — see [GPU rendering](#gpu-rendering-hybrid-intel--nvidia-laptop).

## Contract (identical to Isaac / real)

| Direction | Topic / interface | Provided by |
|---|---|---|
| PUB | `/livox/lidar` `PointCloud2` (frame `livox_frame`) | gz `gpu_lidar` → `ros_gz_bridge` *(needs a render GPU; bridged from gz `/livox/lidar/points`)* |
| PUB | `/livox/imu` `Imu` | gz `imu` sensor → bridge |
| PUB | `/odom` + TF `odom→base_link` | gz `OdometryPublisher` → bridge |
| PUB | `/joint_states` | `ign_ros2_control` + `joint_state_broadcaster` (native ROS2) |
| SUB | `/cmd_vel` `Twist` (vx, vy, wz) | bridge → gz `VelocityControl` (holonomic) |
| both | MoveIt `FollowJointTrajectory` controllers (`left/right_arm`, grippers) | `ign_ros2_control` (native ROS2, no bridge) |

## Install (once)

```bash
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan \
  ros-humble-moveit ros-humble-xacro
# ros-humble-ros-gz pulls in Gazebo Fortress (ignition-gazebo6).
# NOTE: ros-humble-ign-ros2-control is a SEPARATE package and easy to miss —
#       without it the controller_manager never starts.
```

## Build & source

`moz1_description`, `moz1_moveit_config` and `moz1_navigation_bringup` live in the
**same workspace** as the sim packages here, so there is one build and one overlay —
no dependency-ordered sourcing, no `SPIRITAI_DIR`. From the repo root:
```bash
source source_sim.sh build      # colcon build + source (first time / after edits)
source source_sim.sh            # just source (every new terminal)
source source_sim.sh gpu        # + PRIME offload to the NVIDIA dGPU (needed for the lidar)
source source_sim.sh build gpu  # build, source, and offload — the full first-run line
```
> The `gpu` flag folds in `gpu_nvidia.sh`; drop it on a desktop NVIDIA box or after
> `sudo prime-select nvidia`.
> The build uses `--symlink-install`, so edits to the URDF/xacro and the YAML configs
> take effect without rebuilding. If you ever switch to a plain `colcon build`,
> remember a stale `moz1_description` install shadows your URDF edits — the
> `<inertial>`s vanish and you get the joint/controller cascade in
> [Troubleshooting](#troubleshooting-hard-won).

## Running the system (manipulation → navigation)

Two levels. Always `pkill -9 -f 'ign gazebo'` before a fresh sim launch — the gz
server survives Ctrl-C and a leftover one hijacks `/clock`.

### Level 1 — Manipulation (real MoveIt2)

`fix_base:=true` pins the robot upright for arm work and publishes the `world` frame
the MoveIt SRDF's virtual joint needs:
```bash
source source_sim.sh gpu
pkill -9 -f 'ign gazebo'
ros2 launch moz1_sim_gazebo sim_gazebo_moveit.launch.py fix_base:=true
```
Plan and execute from RViz's **MotionPlanning** panel: pick `left_arm` /
`right_arm` / `dual_arm`, drag the interactive marker, **Plan**, then **Execute** —
`move_group` plans, the Gazebo `joint_trajectory_controller`s execute, and the arm
moves in both gz and RViz. Or command a controller directly:
```bash
ros2 control list_controllers        # all 5 => active
ros2 action send_goal /left_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_gripper_joint],
     points: [{positions: [0.0], time_from_start: {sec: 2}}]}}"
```

### Jerry-can depalletising demo

The Moz1 unloads the top-front row of a pallet of HDPE jerry cans onto the two infeed
conveyors of a liquid-filling line — the cell from the humanoid pallet-reach check,
with the container data (TCCC dimensions, UN marking, empty weights) taken from it.

```bash
source source_sim.sh gpu
pkill -9 -f 'ign gazebo'
ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py                  # 4 L, both arms
ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py container:=10 arms:=left
ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py demo:=false      # scene only
ros2 launch moz1_sim_gazebo sim_gazebo_jerrycan.launch.py gui:=true        # + Gazebo's window
```

**RViz is the view; Gazebo runs headless** (as in artc_ranger_xarm6 — the gz window
renders black on some GPUs). RViz shows the robot, the cell and MoveIt's planned
path. The cell is mirrored from the world SDF by `scripts/scene_markers.py`; the
jerry cans follow the live sim through `gz_pose_tf` (`src/gz_pose_tf.cc`), which
reads each can's pose from Gazebo's `dynamic_pose/info` over ign-transport and
broadcasts it as a TF frame `gz_world → jerrycan_*` (ros_gz_bridge drops the model
names from that message, and the only gz-transport Python bindings here are
Harmonic's).

The launch (1) runs `scripts/jerrycan_scene.py`, writing the world and a scene YAML to
`/tmp/moz1_jerrycan_<size>L/`; (2) includes `sim_gazebo_moveit.launch.py` with that
world, the base pinned and one grasp weld per pickable can (`grasp_targets`); (3) starts
`scripts/jerrycan_demo.py`, which waits for the controllers and move_group, then per can:

1. **locate** the can (Gazebo's pose stands in for perception; a can that's been
   knocked > 3 cm out of place is skipped, not grasped blind);
2. open to 0.08 (≈ 63 mm jaw), **OMPL** to 12 cm above the handle, jaw across the bar;
3. **Cartesian** straight down, close to the bar width, grab (`/grasp/<side>/target`
   then `/grasp/<side>/attach`), lift 5 cm and check the can came up;
4. lift clear of the stack, **OMPL** to above a free conveyor slot (filled back to
   front, so the arm never reaches over a placed can), keeping the grasp yaw;
5. **Cartesian** down until the can is 3 mm above the belt; open, release, retreat;
6. **verify** in Gazebo that the can stands on its slot.

Between motions it waits for the arm to stop: the trajectory controller reports
success when the trajectory's *time* is up, and a loaded arm is still catching up.

It ends with `done: N/M jerry cans verified on the conveyors`. A MoveIt planning scene
mirrors the cell (pallet, stack bodies + handles, full-height cans, conveyors); the
held can is attached to the hand so transits avoid the stack.

| arg | default | |
|---|---|---|
| `container` | `4` | jerry can size in litres: `4`, `10`, `20` |
| `layers` | `0` | stack levels; 0 = 3 (4/10 L) or 2 (20 L) |
| `density` | `1.0` | liquid density, g/mL — sets the can mass |
| `physics_step` | `0.004` | gz step, s |
| `demo` | `true` | run the pick-and-place |
| `arms` | `both` | `both` \| `left` \| `right` (left takes the y > 0 cans) |
| `count` | `0` | max cans, 0 = all |
| `speed` | `0.4` | free-space velocity scaling |
| `gui` | **`false`** | Gazebo's own window |
| `rviz` | `true` | the RViz view above (`config/moz1_jerrycan.rviz`) |
| `lidar` / `spawn_delay` | **`false`** / `8.0` | passed to the sim |

`sim_gazebo_moveit.launch.py` gained `world`, `grasp_targets`, `rviz` and
`allowed_start_tolerance` (default 0.01, MoveIt's own) pass-throughs for this.

Regenerate the checked-in `worlds/moz1_jerrycan.world` (4 L, usable with the plain
launches plus `grasp_targets:=left:jerrycan_left_1,...`) with
`scripts/jerrycan_scene.py --out-dir worlds`.

Design notes, each learnt the hard way:

- **Stacked cans collide as a full-height envelope** (W × L × h). With body-only
  collision the can above sat on the body top, 45 mm low with the handle sunk into it,
  and every grasp went 45 mm off.
- **Pick targets have body-only collision.** The jaw has to straddle the handle bar;
  the grasp itself is the grasp plugin, so the handle is visual.
- **The grasp is `KinematicGrasp`, not gz's DetachableJoint** (`src/kinematic_grasp.cc`,
  same parameters and topics). Fortress's dartsim joins two models with a soft
  `WeldJointConstraint`; with a 4.2 kg can it held to the millimetre in some runs and
  let the can swing 20 cm, or flung the arm, in others. KinematicGrasp re-poses the
  held can at its grasped offset every step and puts its weight on the gripper link
  as a wrench, so the hold is rigid and the arm still carries the load.
- **Loaded arms sag.** gz's position control leaves ~0.02 rad of error under a 4 kg
  can, past MoveIt's default 0.01 rad start tolerance; the launch sets
  `allowed_start_tolerance:=0.05` on move_group.
- **Close edge isn't trusted.** The trajectory controller reports success when time
  runs out, reached or not, so the demo requests the grab explicitly after closing
  (grasp_helper's edge trigger stays as the sim/real-agnostic path).
- **Controllers at 250 Hz** (`gazebo_controllers.yaml`), matching the 4 ms physics
  step. At 100 Hz the loaded wrist fell into a limit cycle.
- **Keep the yaw.** Both top-down yaws are valid grasps, but the transit to the slot
  must end at the yaw the straight-line moves use, or the descent would need a
  half-turn of the wrist.
- **Payload is real.** The held can's weight loads the arm, against the URDF's
  50 N·m arm effort limits (Moz1's real torques and payload are unpublished; 5 kg/arm
  is the reach page's placeholder). 4 L (4.2 kg) runs 6/6. 10 L (10.55 kg) goes 0/4:
  the arm sags 16 cm the moment it takes the load (~52 N·m at the shoulder). The
  demo logs the over-rating and tries anyway; a dual-arm lift is the next step.

### Jerry-can transfer demo → `hmlv_cell`

The transfer demo (one pair of cans: pallet A → conveyor + filling → box on pallet B,
both arms, base driving) moved to the shared cell workspace,
[`hmlv_cell/`](../../../hmlv_cell/README.md), so other robots can run it:
`ros2 launch hmlv_cell_gazebo transfer.launch.py [robot:=moz1|g1]`. It still uses this
package's `KinematicGrasp`, `ConveyorBelt`, `gz_pose_tf`, `scene_markers.py`,
`grasp_helper.py` and `jerrycan_demo.py` / `jerrycan_scene.py`, and Moz1's settings
for it (torso poses, gripper, grasp) are in `hmlv_cell_gazebo/robots/moz1.yaml`.

### Level 2 — Navigation (Nav2 + SLAM)

Terminal 1 — sim + Nav2 + SLAM + RViz (**GUI stays ON — required for the lidar**, see note):
```bash
source source_sim.sh gpu
pkill -9 -f 'ign gazebo'
ros2 launch moz1_sim_gazebo sim_gazebo_nav.launch.py
```
Wait ~40 s (controllers ~25 s; Nav2 + SLAM + RViz start ~30 s; then SLAM maps).

Terminal 2 — **required**: feed the costmap's obstacle layer. `nav2_params.yaml` is
shared with the real robot, where the obstacle layer subscribes to FAST-LIO2's
`/cloud_registered_body`; the sim publishes the raw cloud on `/livox/lidar`, so relay it:
```bash
source source_sim.sh
ros2 run topic_tools relay /livox/lidar /cloud_registered_body
```
Send a goal — RViz **2D Nav Goal** button, or CLI (wait for
`ros2 lifecycle get /bt_navigator` → `active` first):
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
> **SLAM, not AMCL.** This sim localizes with `slam_toolbox`, which publishes
> `map→odom` continuously and has **no `amcl/get_state` service**. If you drive Nav2
> from `nav2_simple_commander`, the stock `BasicNavigator.waitUntilNav2Active()` hangs
> forever on `amcl/get_state service not available, waiting...` — wait on
> `bt_navigator` instead, or pass `localizer='bt_navigator'`.

> ⚠️ **The gz GUI must be ON for navigation.** On a hybrid Intel+NVIDIA (PRIME) box the
> **headless** gz server does **not** render the scene for the `gpu_lidar`, so `/scan`
> comes back nearly empty (~2 returns) and SLAM builds no map. `gui:=true` is the nav
> default for this reason — use **RViz** as your working view and just minimise the gz
> window. (Manipulation has no lidar, so it runs fine headless.) The relay above is
> needed because `nav2_params.yaml` (shared with the real robot) expects the FAST-LIO2
> topic `/cloud_registered_body`.

## Run (reference)

```bash
# On a hybrid Intel+NVIDIA laptop, route gz to the NVIDIA GPU FIRST (see below):
source ../gpu_nvidia.sh     # repo root; or: source source_sim.sh gpu
# Full sim (lidar + GUI):
ros2 launch moz1_sim_gazebo sim_gazebo.launch.py
ros2 launch moz1_sim_gazebo sim_gazebo_nav.launch.py       # + Nav2 / SLAM
ros2 launch moz1_sim_gazebo sim_gazebo_moveit.launch.py    # + MoveIt move_group
ros2 launch moz1_sim_gazebo sim_gazebo_full.launch.py      # BOTH: free base + Nav2 + move_group
```
> `sim_gazebo_full` runs the whole pick-and-place DAG (navigate→grasp→navigate→place)
> in **one** sim: free holonomic base (drives) + lidar/Nav2/SLAM + move_group. The
> MoveIt SRDF pins base_link to `world` (fixed virtual joint), so it publishes a
> static `world→map` and relies on the DAG being **sequential** (base parks, then
> the arm plans). See the launch file's docstring for the details/caveats.
**Always `pkill -9 -f 'ign gazebo'` between runs** — the gz server survives Ctrl-C
and a leftover one hijacks `/clock` (controllers then can't activate).

### Launch args

| arg | default | purpose |
|---|---|---|
| `world` | `moz1_pickplace.world` | gz-sim world (has the bearing-ring + bottle fixtures); `moz1_lab.world` = bare scene |
| `lidar` | `true` | `gpu_lidar` sensor (ogre2-rendered → needs a render GPU). `false` disables it (lighter; use if you only need arms/base and want max RTF) |
| `gui` | `true` | gz 3D GUI. `false` = headless — **but the lidar won't render headless on a PRIME laptop**, so keep it ON for nav (fine to disable for arms-only/manipulation) |
| `fix_base` | `false` (but the MoveIt launch defaults it **true**) | pin `base_link` to a `world` link — keeps the robot **upright/stable** and publishes the **`world`** frame the MoveIt SRDF `virtual_joint` needs. `false` = free holonomic base (VelocityControl + `/odom`) |
| `spawn_delay` | `8.0` | seconds after spawn before starting controllers, so the sim is stepping first; raise on slow machines |
| `grasp_targets` | `left:bearing_ring,right:sample_bottle` | `side:model` pairs the grippers can grab; one KinematicGrasp plugin + `/grasp/<side>/<model>/{attach,detach}` bridge per pair. `grasp_helper` grabs the pair selected on `/grasp/<side>/target` (default: the side's first) when the gripper closes |
| `sim_collision` | (set true by the launch) | swap STL collision meshes for fast AABB boxes — see [Design notes](#design-notes) |

## GPU rendering (hybrid Intel + NVIDIA laptop)

gz's ogre2 needs a working render GPU. On a hybrid laptop (Intel iGPU for display,
NVIDIA dGPU for compute, PRIME **on-demand**), gz defaults to the **Intel iGPU** —
and a very new iGPU (e.g. Meteor Lake `0x7d67`) whose Mesa doesn't know the PCI id
fails with `MESA: … does not support the 0x… PCI ID` / `failed to create dri2 screen`.
The fix is **not** to fight Mesa — it's to route gz to the NVIDIA card:

```bash
source ../gpu_nvidia.sh                      # PRIME render-offload env for THIS shell
glxinfo -B | grep "OpenGL renderer"          # must say: NVIDIA … (else stop & fix)
ros2 launch moz1_sim_gazebo sim_gazebo.launch.py    # GUI + lidar now render on NVIDIA
```
`gpu_nvidia.sh` sets the PRIME-offload vars (GLX for the gz 3D window; EGL for sensor
rendering). Source it in every terminal that launches gz.

> **Headless caveat (this hardware):** with `gui:=false` the gz server does **not**
> render the scene for the `gpu_lidar` here — the cloud comes back nearly empty. So
> **navigation needs `gui:=true`** (its default). Control/manipulation (no sensor
> rendering) run fine headless. If your GPU renders headless sensors correctly, you
> can use `gui:=false` for nav too.

**Make it permanent** (no per-terminal sourcing): `sudo prime-select nvidia && sudo reboot`.
After reboot NVIDIA is the default renderer for everything and `gpu_nvidia.sh` becomes
a no-op. Trade-off: higher idle power. Revert with `sudo prime-select on-demand && sudo reboot`.

**Truly GPU-less machine?** Run `lidar:=false gui:=false` (all 5 controllers + base +
odom + imu still come up for manipulation/teleop) and use **Isaac for navigation**
(no lidar → no SLAM here). Bump `spawn_delay:=30` if controllers time out on a slow box.

## Visualize it (RViz)

The gz 3D GUI needs the GPU; **RViz renders separately** (Ogre1/GLX, and it accepts
software GL — so it works where the gz GUI doesn't). With the (headless) sim
running, in another terminal:
```bash
rviz2 -d $(ros2 pkg prefix moz1_sim_gazebo)/share/moz1_sim_gazebo/config/moz1_sim.rviz \
      --ros-args -p use_sim_time:=true
# if it fails to render, force software GL (RViz accepts it, gz doesn't):
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d $(ros2 pkg prefix moz1_sim_gazebo)/share/moz1_sim_gazebo/config/moz1_sim.rviz --ros-args -p use_sim_time:=true
```
RViz reads `/robot_description` + `/tf` + `/joint_states` and shows the robot (full
visual meshes) moving. Drive it and watch:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.1}}'
```
**gz = headless physics engine, RViz = viewer** is a standard split (common even on
good GPUs). For arm motion, run `sim_gazebo_moveit.launch.py gui:=false` and use
RViz's MotionPlanning panel.

## Verify

```bash
ros2 topic echo /clock --once          # returns immediately, sec advancing (sim stepping)
ros2 control list_controllers          # all 5 => active
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'; ros2 topic echo /odom --once
ros2 topic hz /livox/lidar             # ~10 Hz (lower if RTF<1); needs lidar:=true + render GPU
ros2 topic echo /livox/lidar --once | grep frame_id   # => livox_frame (not moz1/base_link/…)
```
> RTF note: on a single laptop, GUI + a 64×360 lidar + physics can run at RTF≈0.5,
> so `/livox/lidar` reports ~5 Hz. That's fine for SLAM. `gui:=false` (+ RViz) or a
> smaller lidar (`<vertical><samples>`) raises it.

## Design notes

- **Holonomic base (VelocityControl — kinematic, not physical):** the mecanum wheels
  are visual-only in `moz1_description`; the base moves via the gz `VelocityControl`
  system, which sets the model's velocity **directly** from `/cmd_vel` (vx,vy,wz) —
  matching Nav2's MPPI-Omni output and keeping the base upright (no tipping under the
  arms). **Known limitation (observed in testing):** because it commands velocity
  directly, the base is *kinematic* — it resists gravity and behaves like an
  unstoppable object (won't fall off edges, can't be pushed, no wheel slip/traction).
  That's an acceptable trade for a nav/manip demo where Nav2 owns the base velocity.
  For **physically-driven** mecanum (torque at the wheels, real slip) you'd replace
  VelocityControl with wheel `ros2_control` joints + a mecanum/omni drive controller.
  Note Gazebo **Fortress has no built-in `MecanumDrive` system** (that's Garden+), so
  it's custom and roller friction is finicky — see e.g.
  [`ros2_omni_robot_sim`](https://github.com/YePeOn7/ros2_omni_robot_sim) (4-wheel omni)
  as a reference. Defer unless locomotion realism is a hard requirement.
- **Inertials:** `moz1_description`'s links had none (built for kinematics). Placeholder
  mass+inertia is now inlined on the moving links (mesh/arm/gripper). Physics-only —
  ignored by RViz/MoveIt/real robot.
- **Collision (`sim_collision`):** the launch passes `sim_collision:=true`, swapping the
  detailed CAD `.STL` collision meshes for **AABB boxes** (`moz1_collision_boxes.xacro`,
  regenerate with the bounding-box script if meshes change) — a large speedup, esp.
  CPU-only. The **real robot / MoveIt keep the STL meshes** (default `false`), so
  planning is unaffected. Boxes are conservative; refine per-link if needed.
- **ros2_control name:** the URDF uses `ign_ros2_control/IgnitionSystem` +
  `ign_ros2_control-system`; the plugin logs `The ign_ros2_control plugin got renamed
  to gz_ros2_control` — harmless on Fortress.
- **Lidar bridge topic:** a gz `gpu_lidar` publishes a `LaserScan` on its bare
  `<topic>` and the **`PointCloudPacked` on `<topic>/points`**. The bridge therefore
  reads gz **`/livox/lidar/points`** and remaps it to the ROS `/livox/lidar` contract
  (`sim_gazebo.launch.py`). Bridging the bare topic silently yields no cloud.
- **Lidar/imu frame:** urdf2sdf collapses the fixed `livox_frame` joint into
  `base_link`, so the sensor would otherwise report `frame_id=moz1/base_link/livox_lidar`.
  `<ignition_frame_id>livox_frame</ignition_frame_id>` on each sensor restores the
  `livox_frame` the TF tree / SLAM / real robot expect.

## Troubleshooting (hard-won)

| Symptom | Cause / fix |
|---|---|
| `package 'moz1_navigation_bringup' not found` (or `'moz1_description' not found`) at launch, then a clean shutdown | the workspace isn't sourced (or wasn't built) — `source source_sim.sh build`. A thrown launch action tears the whole launch down, hence the "shuts down by itself" |
| a `nav2_simple_commander` client hangs on `amcl/get_state service not available, waiting...` | this sim localizes with `slam_toolbox`, not `amcl` — wait on `bt_navigator` (`ros2 lifecycle get /bt_navigator` → `active`) instead of calling `waitUntilNav2Active()` with its default localizer |
| `link[...] has no <inertial>` → `joint[...] ignored`, disconnected-vertex flood | stale `moz1_description` install shadowing your URDF edits — rebuild with `--symlink-install` (`source source_sim.sh build`) |
| `Failed to load system plugin [ign_ros2_control-system]` | install `ros-humble-ign-ros2-control`; the launch also puts its lib dir on `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` |
| `Unable to find … model://moz1_description/meshes/*.STL` | gz needs `IGN_GAZEBO_RESOURCE_PATH` = the share **parent** (the launch sets this) |
| `Found additional publishers on /clock, using namespaced clock topic only` → `/clock` empty, controllers never activate | leftover gz server from a previous run — `pkill -9 -f 'ign gazebo'` and relaunch |
| `Switch controller timed out after 5s` / spawners die | sim not stepping yet (still loading, or GPU render stall) — run headless (`gui:=false`), disable lidar (`lidar:=false`), and/or raise `spawn_delay` |
| `Failed to activate controller` but `list_controllers` shows `active` | cosmetic — a slow switch exceeded the spawner's service timeout and retried an already-active controller (STRICT abort). `list_controllers` is the truth |
| `LIBGL_ALWAYS_SOFTWARE=1` ignored by gz | ogre2 forces a hardware EGL device — you can't software-render the gz view; run headless and use RViz (which *does* accept software GL) |
| `MESA: … does not support the 0x… PCI ID` / `failed to create dri2 screen` (hybrid laptop) | gz picked the Intel iGPU. `source gpu_nvidia.sh` (PRIME offload to the NVIDIA card) or `sudo prime-select nvidia` — see [GPU rendering](#gpu-rendering-hybrid-intel--nvidia-laptop) |
| `/livox/lidar` silent but sim renders & `/cmd_vel` works | bridge read the wrong gz topic — the cloud is on gz `/livox/lidar/**points**`. Fixed in `sim_gazebo.launch.py` (bridge `/livox/lidar/points` → remap `/livox/lidar`) |
| lidar `frame_id: moz1/base_link/livox_lidar` (SLAM/Nav2 can't transform the cloud) | urdf2sdf lumped `livox_frame` into `base_link`; `<ignition_frame_id>livox_frame</ignition_frame_id>` on the sensor restores it. If your Fortress build ignores that tag, override the frame in the bridge instead |
| `Frame [world] does not exist` / `Requesting initial scene failed` / robot spawns **tilted** | the MoveIt SRDF wants a `world` frame, and the free base tips under the (placeholder-mass) arms — run with `fix_base:=true` (the MoveIt launch does by default): it pins the base upright and publishes `world` |
| `/clock` silent, grasp topics dead, but `ign topic -l` shows everything | the ROS bridge is the Harmonic one (`ros-humble-ros-gzharmonic`, gz-transport13) — it can't talk to Fortress. Build ros_gz for Fortress (top-level README, *Install*); `source_sim.sh` overlays it |
| `fix_base:=true` sim crawls (~1 step / 10 s), controllers time out | wheels buried in the ground. The fixed base spawns at z = 0.01 (base_link 0.11, wheels r = 0.105); don't lower it |
| gripper controllers won't activate: `Not existing: [ left_gripper_joint/position ]` | the gripper's actuator link was dropped by urdf2sdf — fixed by `_inject_default_inertials` (movable-joint children get an inertial). If it's back, check that function still runs |
| two demos fighting: arm goals `CONTROL_FAILED`, grasp targets switching by themselves, `TF_OLD_DATA` for `jerrycan_*` frames | nodes from an earlier run are still alive. The gz server *and* the demo / `gz_pose_tf` nodes can survive Ctrl-C — `pkill -9 -f 'ign gazebo'; pkill -f jerrycan_demo; pkill -f transfer_demo; pkill -f gz_pose_tf; pkill -f move_group` |
| a gripper controller stays `inactive`, spawner log: `A controller named … was already loaded` | the spawner's `load_controller` call timed out (>10 s, busy machine), its retry was refused and it exited. The demos activate such controllers themselves after 30 s |
| RViz: `Could not … robot_description_semantic` / can't parse SRDF | the RViz node needs the SRDF — `sim_gazebo_moveit.launch.py` now passes `robot_description` + `robot_description_semantic` to it |
| MoveIt: `The complete state of the robot is not yet known. Missing Base-0..3` | the wheel joints weren't in `ros2_control`; they're now added **state-only** so `/joint_states` is complete |

## Reused unchanged (sim ↔ real parity)

`moz1_description`, `moz1_navigation_bringup` and `moz1_moveit_config` are the same
packages the real Moz1 runs — nothing in them is sim-specific. Only the launch/env
differs between **Gazebo, Isaac, and the real robot**. See the
[repo README](../../README.md) for the contract table and the two-backend overview.
