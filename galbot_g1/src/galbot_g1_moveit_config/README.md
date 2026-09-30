# galbot_g1_moveit_config

MoveIt 2 configuration for the Galbot G1 (One Golf), hand-written (no Setup Assistant
project). Group names follow the Moz1 config so the same task code drives both robots:

| Group | What |
|---|---|
| `left_arm`, `right_arm` | 7 DoF, `<side>_arm_base_link` → `<side>_gripper_tcp_link` (KDL) |
| `dual_arm` | both arms |
| `torso` | `leg_joint1..5` — lifts, leans and turns the whole upper body (joint space only) |
| `head`, `left_gripper`, `right_gripper` | |

Named states: `home` (arms: hands in front of the chest; torso: standing, shoulders at
1.27 m), grippers `open` / `closed`. The base is not a MoveIt joint: `world` → `base_link`
is a fixed virtual joint, so plans are in the base's frame.

Controllers (`moveit_controllers.yaml`) are FollowJointTrajectory: `left_arm_controller`,
`right_arm_controller`, `leg_controller`, `<side>_gripper_controller` — what the transfer
cell's G1 exposes (`hmlv_cell/src/hmlv_cell_gazebo/robots/g1/controllers.yaml`).
`galbot_g1_gazebo` (Harmonic) drives its grippers with GripperCommand instead, so this
config plans for it but can only execute arm and leg motions there.

Acceleration limits are not published by Galbot; `joint_limits.yaml` uses conservative
values (arms 1.0, leg 0.5 rad/s²).

## Regenerating the collision matrix

`disable_collisions` in `config/galbot_g1.srdf` comes from sampling. After changing the
groups or the description, strip the `disable_collisions` lines and run:

```bash
xacro config/galbot_g1.urdf.xacro > /tmp/g1.urdf
ros2 run moveit_setup_assistant collisions_updater --urdf /tmp/g1.urdf \
  --srdf config/galbot_g1.srdf --output config/galbot_g1.srdf \
  --default --always --trials 200000
```

It takes a few seconds. Use 200k samples: 10k and 50k still disable pairs that do
collide (e.g. one arm's wrist against the other arm). A 20k run once hung — if it takes
more than a minute, kill it and rerun. Then restore the header comment and the hand
edits at the end of the file: `<side>_arm_link5` / `<side>_arm_link7` are disabled by
hand (`reason="User"`) because their convex hulls both wrap the 2-axis wrist and
overlap whenever it bends, although the hand can't reach the forearm within the joint
limits. The sampler keeps that pair enabled, and with it no top-down grasp further than
~0.8 m is collision-free.
