#!/usr/bin/env python3
"""scene_markers.py — republish the Gazebo world's static scene as RViz Markers.

Lets you run Gazebo HEADLESS (gui:=false) and still see the world (ground tables,
fixtures) in RViz alongside the RobotModel. It parses the gz-sim `.world` SDF and
publishes one visualization_msgs/Marker per model (box / cylinder / sphere) on
`/scene_markers`, in the `map` frame.

Frame note: the robot spawns at the gz world origin and SLAM anchors `map` there,
so gz world coordinates ≈ map coordinates — the fixtures land in the right place
relative to the robot. Objects are published at their INITIAL world poses; dynamic
objects (the ring/bottle) won't track if grasped/moved (they're a visual reference,
not a live mirror). Latched (transient-local) + re-published so late RViz clients
still get them.
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


class SceneMarkers(Node):
    def __init__(self):
        super().__init__("scene_markers")
        self.world_file = self.declare_parameter("world_file", "").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/scene_markers", qos)
        self.markers = self._build()
        self.get_logger().info(
            f"publishing {len(self.markers.markers)} scene marker(s) from "
            f"{self.world_file} in frame '{self.frame_id}'")
        self.create_timer(2.0, lambda: self.pub.publish(self.markers))
        self.pub.publish(self.markers)

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
            px, py, pz, rr, pp, yy = _floats(model.findtext("pose"), 6) \
                if model.find("pose") is not None else (0.0,) * 6
            vis = model.find("link/visual")
            geom = vis.find("geometry") if vis is not None else None
            if geom is None:
                continue
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = "scene"
            m.id = mid
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, pz
            m.pose.orientation = _rpy_to_quat(rr, pp, yy)
            box, cyl, sph = geom.find("box"), geom.find("cylinder"), geom.find("sphere")
            if box is not None:
                sx, sy, sz = _floats(box.findtext("size"), 3)
                m.type = Marker.CUBE
                m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            elif cyl is not None:
                r = float(cyl.findtext("radius") or 0.1)
                length = float(cyl.findtext("length") or 0.1)
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = 2 * r
                m.scale.z = length
            elif sph is not None:
                r = float(sph.findtext("radius") or 0.1)
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 2 * r
            else:
                continue  # plane / mesh / other → skip (Grid display covers the floor)
            dif = vis.find("material/diffuse")
            r, g, b, a = _floats(dif.text, 4) if dif is not None else (0.6, 0.6, 0.6, 1.0)
            m.color = ColorRGBA(r=r, g=g, b=b, a=a if a > 0 else 1.0)
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
