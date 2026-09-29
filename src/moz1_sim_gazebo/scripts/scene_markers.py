#!/usr/bin/env python3
"""scene_markers.py — republish the Gazebo world's static scene as RViz Markers.

Lets you run Gazebo HEADLESS (gui:=false) and still see the world (ground tables,
fixtures) in RViz alongside the RobotModel. It parses the gz-sim `.world` SDF and
publishes one visualization_msgs/Marker per model (box / cylinder / sphere) on
`/scene_markers`, in the `map` frame.

Frame note: the robot spawns at the gz world origin and SLAM anchors `map` there,
so gz world coordinates ≈ map coordinates — the fixtures land in the right place
relative to the robot. Latched (transient-local) + re-published so late RViz clients
still get them.

Every <visual> of every <link> is drawn, at its link + visual pose. Models are
published at their INITIAL world poses, except those whose name starts with the
`tf_model_prefix` parameter and aren't <static>: they are drawn in a TF frame named
after the model, so they follow the live simulation when something broadcasts that
frame (gz_pose_tf, as the jerry-can launch does). Empty prefix (default) = everything static, as before.
"""
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _rpy_to_quat(r, p, y):
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy)


def _floats(text, n):
    vals = [float(v) for v in (text or "").split()]
    return (vals + [0.0] * n)[:n]


def _rot(r, p, y):
    """rpy (SDF fixed-axis x-y-z) -> 3x3 rotation matrix as nested lists."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _compose(a, b):
    """(xyz, R) poses: a * b."""
    (pa, ra), (pb, rb) = a, b
    p = [pa[i] + sum(ra[i][k] * pb[k] for k in range(3)) for i in range(3)]
    r = [[sum(ra[i][k] * rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return p, r


def _pose(text):
    v = _floats(text, 6)
    return v[:3], _rot(*v[3:])


def _quat(r):
    """3x3 rotation -> Quaternion."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return Quaternion(w=0.25 / s, x=(r[2][1] - r[1][2]) * s,
                          y=(r[0][2] - r[2][0]) * s, z=(r[1][0] - r[0][1]) * s)
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i][i] - r[j][j] - r[k][k])
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j][i] + r[i][j]) / s
    q[k] = (r[k][i] + r[i][k]) / s
    return Quaternion(x=q[0], y=q[1], z=q[2], w=(r[k][j] - r[j][k]) / s)


class SceneMarkers(Node):
    def __init__(self):
        super().__init__("scene_markers")
        self.world_file = self.declare_parameter("world_file", "").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.tf_prefix = self.declare_parameter("tf_model_prefix", "").value
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/scene_markers", qos)
        self.markers = self._build()
        self.get_logger().info(
            f"publishing {len(self.markers.markers)} scene marker(s) from "
            f"{self.world_file} in frame '{self.frame_id}'")
        self.create_timer(2.0, lambda: self.pub.publish(self.markers))
        self.pub.publish(self.markers)

    @staticmethod
    def _marker(vis, pose):
        geom = vis.find("geometry")
        if geom is None:
            return None
        m = Marker()
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pose[0]
        m.pose.orientation = _quat(pose[1])
        box, cyl, sph = geom.find("box"), geom.find("cylinder"), geom.find("sphere")
        if box is not None:
            m.type = Marker.CUBE
            m.scale.x, m.scale.y, m.scale.z = _floats(box.findtext("size"), 3)
        elif cyl is not None:
            m.type = Marker.CYLINDER
            m.scale.x = m.scale.y = 2 * float(cyl.findtext("radius") or 0.1)
            m.scale.z = float(cyl.findtext("length") or 0.1)
        elif sph is not None:
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = 2 * float(sph.findtext("radius") or 0.1)
        else:
            return None  # plane / mesh / other → skip (Grid display covers the floor)
        dif = vis.find("material/diffuse")
        r, g, b, a = _floats(dif.text, 4) if dif is not None else (0.6, 0.6, 0.6, 1.0)
        m.color = ColorRGBA(r=r, g=g, b=b, a=a if a > 0 else 1.0)
        return m

    def _build(self):
        arr = MarkerArray()
        try:
            root = ET.parse(self.world_file).getroot()
        except Exception as exc:
            self.get_logger().error(f"could not parse world '{self.world_file}': {exc}")
            return arr
        world = root.find("world")
        mid = 0
        for model in (world.findall("model") if world is not None else []):
            name = model.get("name", "")
            static = (model.findtext("static") or "").strip() in ("true", "1")
            live = bool(self.tf_prefix) and name.startswith(self.tf_prefix) and not static
            model_pose = ([0.0] * 3, _rot(0, 0, 0)) if live else _pose(model.findtext("pose"))
            for link in model.findall("link"):
                link_pose = _compose(model_pose, _pose(link.findtext("pose")))
                for vis in link.findall("visual"):
                    m = self._marker(vis, _compose(link_pose, _pose(vis.findtext("pose"))))
                    if m is None:
                        continue
                    m.header.frame_id = name if live else self.frame_id
                    # re-transform every frame: otherwise RViz places a marker only
                    # when the array arrives (every 2 s) and moving cans jump
                    m.frame_locked = live
                    m.ns = name
                    m.id = mid
                    arr.markers.append(m)
                    mid += 1
        return arr


def main():
    rclpy.init()
    node = SceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
