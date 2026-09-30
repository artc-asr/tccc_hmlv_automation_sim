# tccc_hmlv_automation_sim

Robot simulations for TCCC HMLV automation (jerry-can handling: depalletising,
filling line, palletising), on **Ubuntu 22.04 / ROS 2 Humble**. Formerly
`spiritai_moz1_simulation`.

| Robot | Directory | Simulator | What it can do |
|---|---|---|---|
| Spirit AI **Moz1** | [`spiritai_moz1/`](spiritai_moz1/README.md) | Gazebo Fortress (+ Isaac Sim) | MoveIt, Nav2/SLAM, jerry-can depalletising; the transfer cell |
| Galbot **G1** (One Golf) | [`galbot_g1/`](galbot_g1/README.md) | Gazebo Harmonic (own sim), Fortress (in the cell) | ros2_control on every joint, holonomic base, lidar + 3 RGB-D cameras, demo motion; MoveIt config; the transfer cell |
| Rainbow Robotics **RB-Y1** (Model A v1.2) | [`rainbowrobotics_rby1/`](rainbowrobotics_rby1/README.md) | Gazebo Harmonic | ros2_control on every joint, diff-drive base, head RGB-D + IMU, demo motion |

Each robot directory is its **own colcon workspace** with its own `source_sim.sh`.
They can't be one workspace: Moz1 runs Gazebo Fortress (with a source-built Fortress
`ros_gz` bridge), G1 and RB-Y1 run Gazebo Harmonic (with `gz_ros2_control` built from
source, shared in [`third_party/`](third_party/README.md)).

The **HMLV cell** ([`hmlv_cell/`](hmlv_cell/README.md)) is the shared work cell — a pallet
of jerry cans, a conveyor with a filling station, a second pallet — that the dual-arm
robots can be dropped into. It is a Fortress workspace overlaying `spiritai_moz1/` (it
reuses Moz1's grasp and conveyor plugins), and it runs the G1 in Fortress too:

```bash
./sim.sh cell                 # transfer demo, Spirit AI Moz1 (default)
./sim.sh cell robot:=g1       # the same demo with the Galbot G1
```

## Quick start

```bash
git clone https://github.com/artc-asr/tccc_hmlv_automation_sim.git
cd tccc_hmlv_automation_sim

./sim.sh                  # list robots and whether they're built
./sim.sh g1 build         # build one workspace (or: ./sim.sh all build)

./sim.sh moz1             # Moz1 in its pick-and-place room
./sim.sh g1               # Galbot G1 at a table
./sim.sh rby1             # RB-Y1 at a table
./sim.sh rby1 gui:=true   # launch args pass through (gui:=true opens Gazebo's GUI)
./sim.sh moz1 sim_gazebo_jerrycan.launch.py   # any launch file of that robot's sim package
./sim.sh cell robot:=g1   # jerry-can transfer cell with the Galbot G1 (default robot: moz1)
./sim.sh g1 gpu           # hybrid Intel+NVIDIA laptop: render on the NVIDIA card

./sim.sh stop             # kill leftover Gazebo servers and sim nodes
```

`sim.sh` launches in a clean subshell with only that robot's workspace sourced. For
working in several terminals (demo scripts, `ros2 topic`, teleop), source the robot's
workspace instead — in a **fresh terminal per robot**:

```bash
cd galbot_g1 && source source_sim.sh
ros2 run galbot_g1_gazebo demo_motion.py
```

| Robot | Default launch | Demo script |
|---|---|---|
| `moz1` | `ros2 launch moz1_sim_gazebo sim_gazebo.launch.py` | `sim_gazebo_jerrycan.launch.py` runs its demo itself |
| `g1` | `ros2 launch galbot_g1_gazebo sim.launch.py` | `ros2 run galbot_g1_gazebo demo_motion.py` |
| `rby1` | `ros2 launch rby1_gazebo rby1_sim.launch.py` | `ros2 run rby1_gazebo rby1_demo.py` |
| `cell` | `ros2 launch hmlv_cell_gazebo transfer.launch.py robot:=moz1\|g1` | runs the transfer demo itself (`demo:=false`: scene only) |

G1 and RB-Y1 default to headless Gazebo with RViz as the viewer (`gui:=true` adds the
Gazebo window, `rviz:=false` drops RViz). Moz1's base launch opens the Gazebo GUI and
no RViz (`gui:=false` for headless); its MoveIt / Nav2 / jerry-can launches and the
cell add RViz.

## Prerequisites

| For | Needs |
|---|---|
| all | ROS 2 Humble, `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-xacro` |
| G1, RB-Y1 | `ros-humble-ros-gzharmonic` (Gazebo Harmonic), `libgz-sim8-dev`, `libgz-plugin2-dev` |
| Moz1, the cell | Gazebo Fortress: `ros-humble-ros-gz` **or**, if `ros-humble-ros-gzharmonic` is installed (it conflicts), a source-built Fortress `ros_gz` in `../gz_fortress_ws` beside this checkout — see [`spiritai_moz1/README.md`](spiritai_moz1/README.md#install-once) |

## Running more than one robot at once

The sims use the same topic names (`/joint_states`, `/cmd_vel`, `/controller_manager` …),
so two at once collide. Give each its own domain in every terminal of that sim:

```bash
export ROS_DOMAIN_ID=42 GZ_PARTITION=g1
```

## Layout

```
sim.sh                  build / launch / stop any robot
gpu_nvidia.sh           PRIME offload of Gazebo rendering to an NVIDIA dGPU (`gpu` flag)
spiritai_moz1/          Moz1 workspace: src/, source_sim.sh, docs/ISAAC.md
galbot_g1/              Galbot G1 workspace: src/, source_sim.sh
rainbowrobotics_rby1/   RB-Y1 workspace: src/, source_sim.sh
hmlv_cell/              the jerry-can cell (Fortress, overlays spiritai_moz1/): transfer demo for any robot
third_party/            gz_ros2_control (humble, built for Harmonic), symlinked into G1 + RB-Y1
```
