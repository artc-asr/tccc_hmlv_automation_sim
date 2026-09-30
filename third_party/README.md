# third_party

## gz_ros2_control

Upstream [ros-controls/gz_ros2_control](https://github.com/ros-controls/gz_ros2_control),
branch `humble` @ `c88a5fd` ("Bump version of pre-commit hooks (#957) (#961)"), unmodified
apart from `COLCON_IGNORE` in `ign_ros2_control`, `gz_ros2_control_demos`,
`ign_ros2_control_demos` and `gz_ros2_control_tests` so only `gz_ros2_control` builds.

Built from source because `ros-humble-gz-ros2-control` from apt targets Gazebo Fortress, while
the Galbot G1 and RB-Y1 sims run Gazebo Harmonic (`ros-humble-ros-gzharmonic`). It must be built
with `GZ_VERSION=harmonic` — each robot's `source_sim.sh build` sets it.

One copy, two users: `galbot_g1/src/gz_ros2_control` and `rainbowrobotics_rby1/src/gz_ros2_control`
are symlinks to this directory, and each workspace builds it into its own `install/`.
The Moz1 (Fortress) workspace does not use it.
