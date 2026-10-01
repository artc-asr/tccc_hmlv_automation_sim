#!/usr/bin/env python3
"""build.py — turn a recorded transfer run into the web replay's data (docs/data/).

    ros2 launch hmlv_cell_gazebo transfer.launch.py rviz:=false record:=/tmp/run_moz1
    tools/web_replay/build.py /tmp/run_moz1            # -> docs/data/moz1/

Reads what hmlv_cell_gazebo's record_run.py wrote (recording.json, robot.urdf,
world.sdf) and writes, for docs/index.html, into docs/data/<robot>/ (the robot
the run was recorded with; one replay per robot, the page switches between them):

    run.json                 samples (joints, base, moving cans) + phases, logs, recolours
    scene.json               the world's boxes / cylinders / spheres (gz world frame)
    robot/robot.urdf         visuals only, mesh paths made relative
    robot/meshes/*.stl       the URDF's meshes, decimated to --face-budget faces in all

and lists the robots that have a replay in docs/data/index.json.

Meshes are found from package:// URIs in the workspaces of this repo (any
directory with that package's package.xml), or absolute file:// paths.
Needs trimesh and fast_simplification (pip install trimesh fast-simplification).
"""
import argparse
import json
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "docs" / "data"
LABELS = {"moz1": "Spirit AI Moz1", "g1": "Galbot G1", "duo": "G1 + Moz1"}
# Per mesh, on top of --face-budget's shared ratio: at most MESH_FACES faces (one
# detailed source part otherwise stays the page's heaviest download), and no loose
# pieces under MIN_PART m (invisible at the page's scale; see drop_specks)
MESH_FACES = 8000
MIN_PART = 0.005
# The duo replay's steps, in three groups the page shows side by side: each robot's
# (record_run.py: "<ns>:<step>" from /<ns>/transfer_demo/phase) and the line's
# (/duo/phase). [group id, title, [(key, label, description), ...]]
DUO_GROUPS = [
    ["g1", "G1 states", [
        ("g1:pick_a", "Pick empties at pallet A", "4 layers, row 4: the torso leans over "
         "the stack and both arms pick a pair at once (0.2 kg each)."),
        ("g1:drive_load", "Carry to the belt", "Hands over the belt slots, then drive to "
         "the load station."),
        ("g1:place_conveyor", "Set the empties on the belt", "Guarded descent: stop 3 cm "
         "short, measure in Gazebo, correct the rest."),
        ("g1:home", "Back to pallet A", "Home pose, then drive back."),
        ("g1:done", "Done", "At pallet A, ready for the next pair.")]],
    ["moz1", "Moz1 states", [
        ("moz1:pick_filled", "Pick the filled pair", "Both arms lift 8.4 kg off the belt's "
         "unload end and curl in; torso to its carry pose."),
        ("moz1:drive_b", "Carry to pallet B", "A gentle drive with the load (0.12 m/s)."),
        ("moz1:place_box", "Place into the box", "Crouch with the arms curled, then reach "
         "over the box on the deck and set both cans down."),
        ("moz1:home", "Back to the unload end", "Home pose, then drive back."),
        ("moz1:done", "Done", "At the unload end, ready for the next filled pair.")]],
    ["line", "Line", [
        ("cycle", "Robots at work", "Two filled cans wait at the belt's unload end, two "
         "empty under the filler; both robots work at once."),
        ("conveyor", "Fill & index", "Both pairs placed: the filler fills the middle pair "
         "(4.2 kg, red), then the belt moves everything one station."),
        ("done", "Line back at its start", "Filled pair at the unload end, the G1's pair "
         "under the filler: ready for the next cycle.")]],
]


# ------------------------------------------------------------------ robot model
def find_package(name, cache={}):
    if name not in cache:
        cache[name] = None
        for xml in REPO.glob("**/package.xml"):
            if "/install/" in str(xml) or "/build/" in str(xml):
                continue
            m = re.search(r"<name>\s*([^<\s]+)\s*</name>", xml.read_text())
            if m and m.group(1) == name:
                cache[name] = xml.parent
                break
    return cache[name]


def resolve_mesh(uri):
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
        root = find_package(pkg)
        return root / rel if root else None
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    return Path(uri)


def drop_specks(mesh, size=MIN_PART):
    """Without the mesh's loose pieces smaller than `size` (m) every way. The Moz1's
    gripper_base.stl is 10,590 pieces, 10,089 of them under 1 mm: the simplifier
    cannot merge across pieces, so it stopped at 22k faces where 6k were asked."""
    parts = mesh.split(only_watertight=False)
    keep = [p for p in parts if p.extents.max() >= size]
    if len(keep) == len(parts) or not keep:
        return mesh
    return trimesh.util.concatenate(keep)


def decimate(mesh, faces):
    import fast_simplification
    if len(mesh.faces) <= faces:
        return mesh
    v, f = fast_simplification.simplify(mesh.vertices.astype(np.float32),
                                        mesh.faces.astype(np.int32),
                                        target_reduction=1.0 - faces / len(mesh.faces))
    return trimesh.Trimesh(v, f, process=True)


def build_robot(urdf_text, out, budget):
    root = ET.fromstring(urdf_text)
    # the page draws visuals only
    for tag in ("gazebo", "ros2_control", "transmission"):
        for el in root.findall(tag):
            root.remove(el)
    for link in root.findall("link"):
        for el in link.findall("collision") + link.findall("inertial"):
            link.remove(el)
    # urdf-loader reads these attributes without URDF's defaults (the G1 has <axis/>)
    for el in root.iter("axis"):
        el.set("xyz", el.get("xyz") or "1 0 0")
    for el in root.iter("origin"):
        for k in ("xyz", "rpy"):
            el.set(k, el.get(k) or "0 0 0")

    meshes = {}                         # uri -> Mesh element(s)
    for m in root.iter("mesh"):
        meshes.setdefault(m.get("filename"), []).append(m)
    loaded = {}
    for uri in meshes:
        path = resolve_mesh(uri)
        if path is None or not path.is_file():
            sys.exit(f"mesh not found: {uri}")
        mesh = trimesh.load(path, force="mesh")
        loaded[uri] = mesh
    total = sum(len(m.faces) for m in loaded.values())
    ratio = min(1.0, budget / total)
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    used, kept = set(), 0
    for uri, mesh in loaded.items():
        stem = Path(uri).stem
        name = stem
        k = 1
        while name.lower() in used:
            k += 1
            name = f"{stem}_{k}"
        used.add(name.lower())
        small = decimate(drop_specks(mesh),
                         min(MESH_FACES, max(200, int(len(mesh.faces) * ratio))))
        kept += len(small.faces)
        small.export(out / "meshes" / f"{name}.stl")
        for el in meshes[uri]:
            el.set("filename", f"meshes/{name}.stl")
    (out / "robot.urdf").write_text(ET.tostring(root, encoding="unicode"))
    print(f"robot: {len(loaded)} meshes, {total} -> {kept} faces")
    return {j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"}


# ------------------------------------------------------------------ scene
def _floats(text, n):
    vals = [float(v) for v in (text or "").split()]
    return (vals + [0.0] * n)[:n]


def _mat(pose_text):
    x, y, z, r, p, yw = _floats(pose_text, 6)
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(yw), math.sin(yw))
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = [x, y, z]
    return m


def _quat(r):
    q = trimesh.transformations.quaternion_from_matrix(r)      # w, x, y, z
    return [round(float(v), 5) for v in (q[1], q[2], q[3], q[0])]


def build_scene(world_file, live):
    """Every visual of every model as a primitive; `live` models in their own frame
    (the page poses them from the recording), the rest in the world frame."""
    world = ET.parse(world_file).getroot().find("world")
    prims = []
    for model in world.findall("model"):
        name = model.get("name", "")
        is_live = name in live
        mp = np.eye(4) if is_live else _mat(model.findtext("pose"))
        for link in model.findall("link"):
            lp = mp @ _mat(link.findtext("pose"))
            for vis in link.findall("visual"):
                g = vis.find("geometry")
                if g is None:
                    continue
                m = lp @ _mat(vis.findtext("pose"))
                if g.find("box") is not None:
                    shape = {"type": "box", "size": _floats(g.findtext("box/size"), 3)}
                elif g.find("cylinder") is not None:
                    shape = {"type": "cylinder", "r": float(g.findtext("cylinder/radius")),
                             "h": float(g.findtext("cylinder/length"))}
                elif g.find("sphere") is not None:
                    shape = {"type": "sphere", "r": float(g.findtext("sphere/radius"))}
                else:
                    continue                                    # the page draws its own floor
                dif = vis.findtext("material/diffuse")
                rgba = _floats(dif, 4) if dif else [0.6, 0.6, 0.6, 1.0]
                prims.append({**shape, "model": name, "visual": vis.get("name", ""),
                              "live": is_live,
                              "pos": [round(float(v), 4) for v in m[:3, 3]],
                              "quat": _quat(m), "rgba": [round(c, 3) for c in rgba]})
    print(f"scene: {len(prims)} primitives, {len(live)} moving models")
    return {"world": world.get("name"), "prims": prims}


# ------------------------------------------------------------------ run
def build_run(rec, movable):
    names = rec["joint_names"]
    keep = [i for i, n in enumerate(names) if n in movable]
    joints = [[row[i] for i in keep] for row in rec["joints"]]
    cans, still = {}, {}
    for name, track in rec["cans"].items():
        a = np.array(track)
        moved = np.abs(a - a[0]).max() > 1e-3
        (cans if moved else still)[name] = track if moved else track[0]
    t = rec["t"]
    print(f"run: {len(t)} samples, {t[-1]:.0f} s, {len(keep)} joints, "
          f"{len(cans)} moving cans, {len(rec['phases'])} phase changes")
    return {"robot": rec.get("robot", ""), "rate": rec["rate"], "t": t,
            "joint_names": [names[i] for i in keep], "joints": joints,
            "base": rec["base"], "cans": cans, "cans_still": still,
            "phases": rec["phases"], "logs": rec["logs"], "recolors": rec["recolors"]}


def build_duo(a, rec):
    """Two robots: each model under robot_<ns>/, one run.json with a `robots` list."""
    robots = []
    budget = a.face_budget // len(rec["robots"])
    for ns, r in rec["robots"].items():
        movable = build_robot((a.run_dir / f"robot_{ns}.urdf").read_text(),
                              a.out / f"robot_{ns}", budget)
        keep = [i for i, n in enumerate(r["joint_names"]) if n in movable]
        robots.append({"id": ns, "label": LABELS.get(ns, ns), "dir": f"robot_{ns}/",
                       "joint_names": [r["joint_names"][i] for i in keep],
                       "joints": [[row[i] for i in keep] for row in r["joints"]],
                       "base": r["base"]})
    run = build_run(dict(rec, joint_names=[], joints=[], base=[]), set())
    run.update(robot="duo", robots=robots, joint_names=[], joints=[], base=[],
               groups=[{"id": g, "title": title, "defs": [list(d) for d in defs],
                        "fail": "failed" if g == "line" else f"{g}:failed"}
                       for g, title, defs in DUO_GROUPS])
    return run


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path, help="default: docs/data/<robot>")
    ap.add_argument("--face-budget", type=int, default=120_000,
                    help="robot mesh faces in all (default 120k, ~6 MB of STL)")
    a = ap.parse_args()

    rec = json.loads((a.run_dir / "recording.json").read_text())
    a.out = a.out or DATA / (rec.get("robot") or "robot")
    if a.out.exists():
        shutil.rmtree(a.out)
    if "robots" in rec:
        for ns in rec["robots"]:
            (a.out / f"robot_{ns}").mkdir(parents=True)
        run = build_duo(a, rec)
    else:
        (a.out / "robot").mkdir(parents=True)
        movable = build_robot((a.run_dir / "robot.urdf").read_text(), a.out / "robot",
                              a.face_budget)
        run = build_run(rec, movable)
    scene = build_scene(a.run_dir / "world.sdf", set(rec["cans"]))
    (a.out / "run.json").write_text(json.dumps(run, separators=(",", ":")))
    (a.out / "scene.json").write_text(json.dumps(scene, separators=(",", ":")))
    size = sum(f.stat().st_size for f in a.out.rglob("*") if f.is_file())
    print(f"wrote {a.out} ({size / 1e6:.1f} MB)")
    write_index()


def write_index():
    """docs/data/index.json: the robots with a replay, Moz1 first, each with its
    model dirs (so the page can fetch the models while run.json downloads)."""
    order = list(LABELS)
    robots = sorted((d.name for d in DATA.iterdir() if (d / "run.json").is_file()),
                    key=lambda r: (order.index(r) if r in order else len(order), r))
    (DATA / "index.json").write_text(json.dumps(
        {"robots": [{"id": r, "label": LABELS.get(r, r),
                     "models": sorted(f"{u.parent.name}/" for u in (DATA / r).glob("*/robot.urdf"))}
                    for r in robots]}, indent=1))
    print(f"index: {', '.join(robots)}")


if __name__ == "__main__":
    main()
