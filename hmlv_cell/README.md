# hmlv_cell — the jerry-can work cell, any robot (Gazebo Fortress)

The TCCC HMLV filling-line cell as a place to drop the dual-arm robots into: pallet A
stacked with empty 4 L jerry cans, a conveyor that carries cans through a filling station
(they come out 4.2 kg), and pallet B with an open box. The **transfer demo** moves one pair
through it with both arms at once, the base driving between stations:

```bash
cd hmlv_cell
source source_sim.sh build                                   # once (after ../spiritai_moz1 is built)
ros2 launch hmlv_cell_gazebo transfer.launch.py              # Spirit AI Moz1 (default)
ros2 launch hmlv_cell_gazebo transfer.launch.py robot:=g1    # Galbot G1
ros2 launch hmlv_cell_gazebo transfer.launch.py demo:=false  # the cell only
```

or from the repo root: `./sim.sh cell [robot:=g1]`. Before relaunching, `../sim.sh stop`:
a leftover Gazebo server or demo node hijacks the next run.

| arg | default | |
|---|---|---|
| `robot` | `moz1` | `moz1` \| `g1` — a profile in `robots/<robot>.yaml` |
| `cleared_rows` | profile (Moz1 `4`, G1 `3`) | top-layer rows of pallet A already gone; the pair comes from the next row (Moz1: row 5, 0.9 m out, it has to lean or reach) |
| `layers` | profile (Moz1 `3`, G1 `4`) | stack height on pallet A |
| `demo` | `true` | run the sequence |
| `speed` | `0.4` | free-space velocity scaling |
| `gui` / `rviz` | `false` / `true` | Gazebo's own window / the RViz view |
| `spawn_delay` | `8.0` | s between the spawn and the controller spawners |

## The sequence

| station (base y) | what happens |
|---|---|
| **pallet A** (0.0) | both arms pick two **empty** cans (0.2 kg) at once: top layer, first remaining row, innermost pair both arms reach, with the least torso change that works (collision-aware IK) |
| **conveyor load** (1.3) | both cans set on the belt side by side |
| conveyor | carries them 2.2 m to the filling station; each becomes **4.2 kg** and turns red (`ConveyorBelt` plugin) |
| **conveyor unload** (3.5) | both arms pick the **filled** pair, curl in toward the chest, torso to its carry pose |
| **pallet B** (5.3) | torso down with the arms curled, then both cans placed into the box on the deck (G1: the box on the floor, no pallet) |

It ends with `done: both filled jerry cans verified in the box on pallet B`; every
placement is checked against Gazebo's own model poses. RViz is the view (Gazebo runs
headless): `scene_markers.py` draws the cell from the world file and `gz_pose_tf` moves
the cans.

## Layout and how the robots plug in

This workspace **overlays `../spiritai_moz1`** (`source_sim.sh` sources it first, with
the Fortress `ros_gz` bridge): the cell reuses `moz1_sim_gazebo`'s Fortress systems —
`KinematicGrasp` (rigid grasp), `ConveyorBelt`, `gz_pose_tf` — and its
`grasp_helper.py`, `scene_markers.py`, `jerrycan_scene.py` (can and pallet geometry) and
`jerrycan_demo.py` (MoveIt plumbing). The G1's description and MoveIt config are
symlinked in from `../galbot_g1/src`.

```
src/hmlv_cell_gazebo/
  launch/transfer.launch.py     the cell + robot:=… (Moz1: moz1_sim_gazebo's sim_gazebo_moveit.launch.py)
  launch/g1_sim.launch.py       Galbot G1 in Fortress + move_group (galbot_g1_moveit_config)
  robots/moz1.yaml, g1.yaml     robot profiles: joints, torso poses, gripper, grasp, footprint
  robots/g1/                    the G1's Fortress model (xacro) and controllers
  scripts/transfer_scene.py     generates the world + scene YAML, laid out from the robot's chassis front
  scripts/transfer_demo.py      the sequence (extends moz1_sim_gazebo's jerrycan_demo.py)
  scripts/reach_sweep.py        offline: which torso poses reach which pallet-A row / the box, and tip or not
src/galbot_one_golf_description -> ../../galbot_g1/src/galbot_one_golf_description
src/galbot_g1_moveit_config     -> ../../galbot_g1/src/galbot_g1_moveit_config
```

To add a robot: a MoveIt config with `left_arm` / `right_arm` / `dual_arm` / `torso`
groups and `<side>_gripper_tcp_link` tips, a Fortress model with FollowJointTrajectory
controllers (`<side>_arm_controller`, `<side>_gripper_controller`, a torso controller),
`<side>_gripper_base_link` for the grasp plugin, a free holonomic base on `/cmd_vel` →
`/odom`, then `robots/<robot>.yaml` and `launch/<robot>_sim.launch.py`.

## The robots

| | Spirit AI Moz1 | Galbot G1 |
|---|---|---|
| model | `moz1_sim_gazebo` (Fortress) | `robots/g1/galbot_g1_fortress.urdf.xacro` (Galbot's description, Fortress plugins) |
| MoveIt | `moz1_moveit_config` | `galbot_g1_moveit_config` |
| chassis front / base_link height | 0.321 m / 0.105 m | 0.307 m / 0.032 m |
| torso | 6 joints, placeholder 100 N·m | 5-joint leg, 280-420 N·m |
| arm efforts | 50 N·m (URDF placeholder) | 180/180/60/60/30/30/30 N·m (Galbot's MuJoCo force ranges) |
| pallet A | 3 layers, row 5: torso `reach70`/`reach80` (hips + waist forward) | 4 layers, row 4 (handles 1.09 m up, 0.79 m out): leg `lean` — the highest and furthest it picks without tipping |
| carry | `carry` (hips back): the loaded torso otherwise folds | standing |
| pallet B box | on the deck: `low_lean` crouch | on the floor (no pallet B): `crouch` (upright, 0.25 m lower) |

### Results

<!-- G1_RESULTS -->

## What it took (Moz1, 2026-09-29/30), for anyone extending it

- **Picks come out of a packed row by sliding back**, not lifting high: lift 8 cm
  (clear of the layer below), then straight back toward the robot. The neighbours
  are 4 mm away on either side; a high lift over them put the hands at the edge of
  their reach, and a free-space path out clipped them.
- **Two arms moving together are collision-checked together.** Each arm's straight
  line is planned alone; if that fails only because the other arm was held still
  (a can rising past the other, still-low wrist camera), the synchronized motion is
  re-checked state by state with `/check_state_validity`.
- **IK is chosen for the whole straight corridor** (random seeds until one
  configuration covers down-to-the-handle and up-to-the-lift): KDL's first answer can
  run an elbow into a joint limit halfway up.
- **Straight lines near full reach can swing the wrist** through a singularity (2-4 rad
  over 0.4 m); moves whose joint travel exceeds 1.5 rad go joint-space instead.
- **Carrying the filled pair (8.4 kg) needs posture on the Moz1.** With its torso
  upright and the hands 0.53 m out, the hip joint needs ~81 of its 100 N·m (URDF
  placeholder) and the torso folded mid-drive. The `carry` pose (hips back, chest
  upright) brings it to ~48 N·m; the drive still stops at 2.5° of torso sag, lets it
  settle and continues (`torso sagged …: stopped, it settled back`). Placing on the deck
  crouches (`low_lean`); not `deep`: loaded, the arms couldn't hold the path there.
- **Guarded placement.** The descent stops 3 cm above the surface, measures where each
  can's bottom really is in Gazebo and corrects the rest, so a sagging arm can't push a
  held can into the belt or the deck.
- **Drives are acceleration-limited** (gz's base plugin applies the commanded velocity
  instantly) and stop within 2 cm / 1°; everything after uses the measured base pose.

## The G1 in Fortress — notes

- **Why Fortress:** the cell's plugins are Fortress systems. The G1's collision geometry
  is STL + spheres, which Fortress loads; its `.glb` visuals only matter to RViz.
  `galbot_g1/` keeps the Harmonic sim (sensors, GripperCommand grippers).
- **Grippers start at 0.6, not on the 0.0 end stop:** started on the limit, the right
  gripper never moved in Fortress (and stalled at first in Harmonic).
- **Wrist self-collision:** `<side>_arm_link5` / `link7`'s convex hulls both wrap the
  2-axis wrist and overlap as soon as it bends; the SRDF disables that pair (see
  `galbot_g1_moveit_config/README.md`). With it, no top-down grasp past ~0.8 m was valid.
- **Grasp geometry:** the G1's tcp approaches along its +X (the Moz1's along +Z), its
  fingers reach 48 mm below the tcp, so it grasps 2 cm above the bar's centre
  (`grasp_z_offset`) to keep the fingertips off the can body; the gripper closes from 0
  (open, 124 mm) to 1.703, the other way round from the Moz1's stroke (`grasp_helper`
  takes the thresholds from the profile).
- **It tips.** Galbot's model is 95 kg with only 23 kg in the chassis, and `leg_joint1`
  swings 17 kg of leg forward; the wheels touch down at x = ±0.176 m. Leaning over
  row 5 of a 4-layer stack (`reach_lean`, upper body fully forward) put the centre of
  mass at x = 0.24 m: the robot tipped 4.4° onto its front wheels and slid 6 cm back,
  and every move planned for a level base was off (the fingers clipped row 6). So
  `reach_sweep.py` checks the centre of mass as well as reach (behind x = 0.15 m, the
  cans in hand included), and the G1 picks row 4 with `lean` (0.138 m). Row 5 is in
  reach but only tipping; a 5th layer is out of reach altogether.
- **The lean over the stack is planned with the arms** (`torso_dual_arm`, a group added
  to the SRDF for this): every IK solution for the pre-grasp was a ~200° swing from home
  that ran a hand through a neighbouring can, and leaning with the arms at home drives
  the hands into the 4-layer stack.
- **Over the box the hands keep their grasp yaw** (`keep_grasp_yaw`): the handle is at
  one end of the can, and the other yaw — a wrist half-turn — swung one filled can into
  the other.
- **Effort limits** are raised at launch to the actuator force ranges of Galbot's MuJoCo
  model, as `galbot_g1_gazebo` does (the URDF's are too weak to hold the leg up).
