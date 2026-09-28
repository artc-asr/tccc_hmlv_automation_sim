# moz1_description/meshes

These `*.STL` files are the **official Moz1 CAD** meshes, copied from
`../../../../spirit01_model/meshes/` so this package is self-contained.

- Body/arm/wheel/head meshes (`base_link.STL`, `left01…left07.STL`,
  `right01…right07.STL`, `leg0N.STL`, `waist0N.STL`, `head2N.STL`, `wheel.STL`)
  are real geometry referenced by `urdf/moz1.urdf.xacro`.
- All four wheels share `wheel.STL`.

## Gripper meshes (`gripper_*.STL`)

Real CAD for the AgiBot / OmniPicker gripper, from the vendor's own gripper
package — the *robot* CAD (`spirit01_model`) still contains no gripper, so these
come from a separate delivery. Referenced by `urdf/moz1_gripper_macros.xacro`.

- The `gripper_` prefix is **required**: the vendor gripper's own body mesh is
  named `base_link.STL` and would otherwise overwrite the robot chassis mesh
  above.
- `gripper_wide3.STL` and `gripper_wide_loop.STL` are **baked Y-mirrors** of the
  narrow parts, produced by `scripts/mirror_stl.py`. Mirroring is baked into real
  files rather than expressed as `scale="1 -1 1"`, because negative mesh scale is
  unreliable outside RViz (Gazebo and several loaders ignore it or invert
  normals). Two vendor data defects are corrected this way — see
  `docs/cpp_sdk_migration.md` §12.34.

## Wrist-camera meshes (`wrist_camera_*.STL`)

Vendor CAD for the wrist Intel RealSense D405 and its clamp bracket, converted
from the supplied STEP files with `scripts/step_to_stl_occ.py`. In **metres**,
like every other mesh here, so the URDF needs no `scale=`. Referenced by
`urdf/moz1_wrist_camera_macros.xacro`; the measurements that place them are
recorded there and in `docs/cpp_sdk_migration.md` §12.36.

- `wrist_camera_mount.STL` — the clamp saddle. Its Ø57.50 mm bore grips the
  gripper body's measured Ø57.28 mm cylinder.
- `wrist_camera_d405.STL` — Intel's own CAD, 42.0 x 23.0 x 42.0 mm. Used for
  **visual only**: the URDF gives the camera a plain box collision instead,
  because this mesh is 42 809 triangles of internal detail.

The remaining sensor frames (head and torso cameras, the EE tips) are still
placeholder primitives defined inline in the xacro, not meshes.

To refresh from CAD:
```bash
cp ../../../../spirit01_model/meshes/*.STL .
```
