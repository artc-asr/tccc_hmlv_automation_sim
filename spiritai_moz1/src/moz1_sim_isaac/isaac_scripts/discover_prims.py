"""
discover_prims.py  --  Moz1 Isaac Sim stage introspection (Phase 1).

Paste this whole file into Isaac Sim 5.1.0 >> Window > Script Editor and Run,
AFTER the Spirit AI Moz1 scene has loaded. No file paths, no imports beyond
omni.* / pxr (already available inside Isaac Sim).

It prints, for the loaded stage:
  * every Articulation root
  * the Moz1 articulation root (heuristic) and its prim path prefix
  * EVERY joint under that articulation, with: joint name, USD type
    (Revolute / Prismatic / Fixed / D6), and whether it is actuated
    (has a drive) -- this is the authoritative list of sim joint names
  * all cameras and any RTX/physics Lidar prims

Why this matters: the real-robot stack expects canonical joint names
(Base-0..3, LegWaist-0..5, LeftArm-0..6, RightArm-0..6, left/right_gripper_joint).
The sim USD may use different names (e.g. Base_0 with an underscore). The
"REMAP SKELETON" block at the end prints a YAML stub mapping each discovered
movable joint to a best-guess canonical name -- copy it into
moz1_sim_bridge/config/joint_name_map.yaml and hand-correct the guesses.
"""

from pxr import Usd, UsdGeom, UsdPhysics
import omni.usd


# Canonical joint names the real Nav2/MoveIt stack expects (for guess-matching).
CANONICAL = [
    "Base-0", "Base-1", "Base-2", "Base-3",
    "LegWaist-0", "LegWaist-1", "LegWaist-2", "LegWaist-3", "LegWaist-4", "LegWaist-5",
    "LeftArm-0", "LeftArm-1", "LeftArm-2", "LeftArm-3", "LeftArm-4", "LeftArm-5", "LeftArm-6",
    "RightArm-0", "RightArm-1", "RightArm-2", "RightArm-3", "RightArm-4", "RightArm-5", "RightArm-6",
    "left_gripper_joint", "right_gripper_joint",
]


def _norm(name):
    """Lowercase, strip separators -- so 'Base_0' == 'Base-0' == 'base0'."""
    return name.lower().replace("_", "").replace("-", "")


def find_articulation_roots(stage):
    return [p.GetPath().pathString for p in stage.Traverse()
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


def guess_robot_root(roots):
    for r in roots:
        low = r.lower()
        if "moz" in low or "spirit" in low:
            return r
    return roots[0] if roots else None


def joint_kind(prim):
    if prim.IsA(UsdPhysics.RevoluteJoint):
        return "Revolute"
    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "Prismatic"
    if prim.IsA(UsdPhysics.FixedJoint):
        return "Fixed"
    if prim.IsA(UsdPhysics.Joint):
        return "D6/Generic"
    return "?"


def has_drive(prim):
    # A drive on either the angular or linear axis means the joint is actuated.
    for axis in ("angular", "linear"):
        if prim.HasAPI(UsdPhysics.DriveAPI) or \
           prim.GetAttribute(f"drive:{axis}:physics:targetVelocity").IsValid() or \
           prim.GetAttribute(f"drive:{axis}:physics:targetPosition").IsValid():
            return True
    return False


def find_joints(stage, root_path):
    """All joint prims under the articulation root, in stage order."""
    joints = []
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return joints
    # Joints often live in a sibling '/joints' scope or as descendants; scan the
    # whole articulation subtree from the root's parent to be safe.
    scan_from = root.GetParent() if root.GetParent().IsValid() else root
    for prim in Usd.PrimRange(scan_from):
        if prim.IsA(UsdPhysics.Joint):
            joints.append(prim)
    return joints


def guess_canonical(sim_name):
    n = _norm(sim_name)
    for c in CANONICAL:
        if _norm(c) == n:
            return c
    return ""  # no confident guess -- human fills in


def main():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[discover] No stage loaded -- open the Spirit Moz1 scene first.")
        return

    roots = find_articulation_roots(stage)
    robot = guess_robot_root(roots)

    print("=" * 72)
    print("Moz1 stage discovery")
    print("=" * 72)
    print(f"Articulation roots ({len(roots)}):")
    for r in roots:
        mark = "  <-- likely Moz1" if r == robot else ""
        print(f"  - {r}{mark}")

    cams = [p.GetPath().pathString for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]
    print(f"\nCameras ({len(cams)}):")
    for c in cams:
        print(f"  - {c}")

    # Lidar prims are typed via the RTX sensor schema; match by name as a fallback.
    lidars = [p.GetPath().pathString for p in stage.Traverse()
              if "lidar" in p.GetName().lower() or "livox" in p.GetName().lower()]
    print(f"\nLidar-like prims ({len(lidars)}):")
    for l in lidars:
        print(f"  - {l}")

    if not robot:
        print("\n[discover] No articulation root found -- cannot enumerate joints.")
        return

    joints = find_joints(stage, robot)
    movable = [j for j in joints if joint_kind(j) in ("Revolute", "Prismatic", "D6/Generic")
               and joint_kind(j) != "Fixed"]

    print("\n" + "=" * 72)
    print(f"Joints under {robot} ({len(joints)} total, {len(movable)} movable):")
    print("=" * 72)
    print(f"  {'JOINT NAME':<28} {'TYPE':<12} {'ACTUATED':<9} GUESS->CANONICAL")
    for j in joints:
        name = j.GetName()
        kind = joint_kind(j)
        act = "yes" if has_drive(j) else "no"
        guess = guess_canonical(name) if kind != "Fixed" else ""
        print(f"  {name:<28} {kind:<12} {act:<9} {guess}")

    # ---- REMAP SKELETON: paste into joint_name_map.yaml and hand-correct ----
    print("\n" + "=" * 72)
    print("REMAP SKELETON  (copy into moz1_sim_bridge/config/joint_name_map.yaml)")
    print("=" * 72)
    print("# sim_joint_name: canonical_urdf_name   (blank guess => FILL IN)")
    print("sim_to_canonical:")
    for j in movable:
        sim = j.GetName()
        guess = guess_canonical(sim)
        if guess:
            print(f'  "{sim}": "{guess}"')
        else:
            print(f'  "{sim}": ""   # TODO: map to one of {CANONICAL}')
    print("\n[discover] Done. Verify the movable count == 26 (4 wheels + 6 torso "
          "+ 7 + 7 arms + 2 grippers) for the full dual-arm Moz1.")


main()
