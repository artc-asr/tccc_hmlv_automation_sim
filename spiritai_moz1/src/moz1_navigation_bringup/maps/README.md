# Maps

Saved 2D occupancy grids live here. Each map is a pair: `<name>.yaml` (metadata) +
`<name>.pgm` (grayscale image).

`lab.pgm` / `lab.yaml` is a grid captured in the real Spirit AI lab — useful as a
reference/regression target, but the simulation does **not** load it: both
`sim_gazebo_nav.launch.py` and `sim_gazebo_full.launch.py` run `slam_toolbox` in
online-async mapping mode and build the map live from the simulated lidar.

To save the map the sim just built:

```bash
ros2 run nav2_map_server map_saver_cli -f src/moz1_navigation_bringup/maps/<name>
```

Run from the workspace root. The trailing `<name>` is the *prefix* —
`map_saver_cli` writes `<name>.yaml` and `<name>.pgm`.
