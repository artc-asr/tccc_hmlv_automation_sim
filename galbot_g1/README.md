# Galbot G1 simulator — ROS 2 Humble + Gazebo Harmonic

Part of [tccc_hmlv_automation_sim](../README.md). Everything below is relative to
this directory (`galbot_g1/`), which is its own colcon workspace.

Simulation of the Galbot G1 (One Golf variant) wheeled dual-arm robot:
ros2_control on every actuated joint, a holonomic base, a Mid-360 lidar and three RGB-D cameras.

| Package | What it is |
| --- | --- |
| `src/galbot_one_golf_description` | Galbot's official URDF/meshes, vendored unmodified (Apache-2.0, see `UPSTREAM.txt`) |
| `src/gz_ros2_control` | symlink to [`../third_party/gz_ros2_control`](../third_party/README.md): `humble` branch, built from source against Gazebo **Harmonic** (the apt package targets Fortress) |
| `src/galbot_g1_gazebo` | This simulator: Gazebo xacro, controllers, sensors, world, launch, demo |
| `src/galbot_g1_moveit_config` | MoveIt 2 config (`left_arm`, `right_arm`, `dual_arm`, `torso` = the leg, grippers); see its [README](src/galbot_g1_moveit_config/README.md) |

The G1 also runs in the shared jerry-can cell, [`../hmlv_cell`](../hmlv_cell/README.md)
(Gazebo **Fortress**, with this description and MoveIt config symlinked in):
`ros2 launch hmlv_cell_gazebo transfer.launch.py robot:=g1`. The Harmonic sim here drives
its grippers with GripperCommand, so `galbot_g1_moveit_config` can plan for it but only
execute arm and leg motions.

## Build

```bash
cd tccc_hmlv_automation_sim/galbot_g1
source source_sim.sh build  # colcon build with GZ_VERSION=harmonic (required by gz_ros2_control)
source source_sim.sh        # every new terminal (use a fresh one, not a Moz1 / RB-Y1 shell)
```

Or from the repo root without sourcing anything: `./sim.sh g1 build`, then `./sim.sh g1 [args]`.

Requires `ros-humble-ros-gzharmonic`, `ros-humble-ros2-control`, `ros-humble-ros2-controllers`
and `ros-humble-xacro` (all installed on this machine).

## Run

Gazebo runs as a headless physics server; **RViz is the viewer**. It shows the robot, TF, lidar,
camera images and the world's models, which `world_markers.py` mirrors from the SDF (the objects on
the table follow the live simulation). The pattern is the same one `artc_ranger_xarm6` uses.

```bash
ros2 launch galbot_g1_gazebo sim.launch.py                # headless Gazebo + RViz
ros2 launch galbot_g1_gazebo sim.launch.py gui:=true   # also open Gazebo's own GUI window
ros2 run galbot_g1_gazebo demo_motion.py                  # moves leg, head, arms, grippers, base
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # drive the base (holonomic: use shift for strafing)
```

Launch arguments: `world`, `gui` (default false), `rviz` (default true), `sensors` (false skips the
lidar/cameras for speed), `x`, `y`, `yaw`.

If another ROS 2 / Gazebo simulation runs on the same machine, isolate them or they will cross-talk:

```bash
export ROS_DOMAIN_ID=42 GZ_PARTITION=galbot
```

## Interfaces

| Part | Joints | Interface |
| --- | --- | --- |
| Leg (torso lift) | `leg_joint1..5` | `/leg_controller/follow_joint_trajectory` |
| Head | `head_joint1` (yaw), `head_joint2` (pitch, + = down) | `/head_controller/follow_joint_trajectory` |
| Arms | `{left,right}_arm_joint1..7` | `/{left,right}_arm_controller/follow_joint_trajectory` |
| Grippers | `{left,right}_gripper_joint` (0 open … 1.703 closed) | `/{left,right}_gripper_controller/gripper_cmd` (GripperCommand) |
| Base | holonomic `vx, vy, wz` | `/cmd_vel` in, `/odom` + TF `odom → base_link` out |

The torso stays upright while `leg_joint3 = leg_joint2 - leg_joint1`; the home stance is `(0.6, 1.8, 1.2, 0, 0)`.

| Sensor | Topics | Frame |
| --- | --- | --- |
| Mid-360 lidar (chassis) | `/lidar/points` | `chassis_lidar_sensor` |
| Head RGB-D | `/head_camera/{image,depth_image,camera_info,points}` | `head_camera_optical_frame` (points: `head_camera_link`) |
| Wrist D405 RGB-D ×2 | `/{left,right}_wrist_camera/{image,depth_image,camera_info,points}` | `<side>_wrist_camera_optical_frame` (points: `<side>_wrist_camera_sensor_link`) |

Also: `/joint_states`, `/robot_description`, `/tf`, `/tf_static`, `/clock`, `/world_markers`
(RViz `MarkerArray` in `odom`, which coincides with Gazebo's world frame).

## Simulation choices

- **Base**: the four omni wheels and their rollers are fixed; the chassis is driven kinematically from
  `/cmd_vel` (gz `VelocityControl`) with frictionless chassis colliders. This follows the commanded twist
  exactly and avoids unstable roller contact physics.
- **Actuator limits**: the URDF effort limits are too weak for Gazebo's joint servos to hold the leg up.
  At launch the effort limits are raised to the actuator force ranges from Galbot's own MuJoCo config
  (`galbot_one_golf_description/config/mjcf/joint_data.json`). Leg goals have a 0.05 rad goal tolerance,
  so a sagging leg reports failure.
- **Gripper linkage**: only `<side>_gripper_joint` is actuated. gz_ros2_control drives the other five
  linkage joints as mimics, and robot_state_publisher computes their TF from the URDF `<mimic>` tags.
- **Physics**: DART with a 2 ms step (about 0.93× real time with all sensors on an RTX 3080 Ti laptop,
  and faster with `sensors:=false`).
- **Camera point clouds**: Gazebo expresses these in the camera body frame, so `pointcloud_frame_relay`
  re-stamps them. Images and camera_info use the ROS optical frame.
