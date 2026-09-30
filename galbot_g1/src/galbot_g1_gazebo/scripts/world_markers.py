#!/usr/bin/env python3
"""Mirror a Gazebo world's models into RViz as a MarkerArray.

Gazebo runs headless and RViz is the viewer, but RViz only draws what is on ROS topics.
This node parses the same SDF world gz-sim loaded and publishes one marker per <visual>
(box, cylinder, sphere, and package:// or file:// meshes), coloured from its <material>.
Static models are drawn at their SDF pose; non-static ones (the objects on the table)
follow the live simulation through gz-sim's /world/<name>/dynamic_pose/info.
Ground planes larger than MAX_FLOOR_SIZE are skipped, so RViz's grid stays visible.

Markers are published in the odom frame, which is gz-sim's world frame
(the OdometryPublisher reports the absolute world pose).

Adapted from ranger_xarm6_description/scripts/world_markers.py (artc_ranger_xarm6).
"""
import threading
import xml.etree.ElementTree as ET

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf_transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

# Largest ground plane (m, either side) drawn as a floor.
MAX_FLOOR_SIZE = 50.0
FLOOR_THICKNESS = 0.01


def pose_matrix(text):
    """SDF '<pose>x y z roll pitch yaw</pose>' -> 4x4 homogeneous matrix."""
    v = [float(x) for x in (text or '').split()] + [0.0] * 6
    m = euler_matrix(v[3], v[4], v[5], 'sxyz')
    m[:3, 3] = v[:3]
    return m


def color_of(visual):
    for tag in ('material/diffuse', 'material/ambient'):
        text = visual.findtext(tag)
        if text:
            return ([float(x) for x in text.split()] + [1.0])[:4]
    return [0.7, 0.7, 0.7, 1.0]


class WorldMarkers(Node):

    def __init__(self):
        super().__init__('world_markers')
        world_file = self.declare_parameter('world_file', '').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        rate = self.declare_parameter('rate', 10.0).value  # [Hz] republish for moving models

        world = ET.parse(world_file).getroot().find('world')
        gz_world = world.get('name')

        # model name -> (world pose 4x4, [(marker, pose in model frame)])
        self.models = {}
        self.dynamic = set()
        self.lock = threading.Lock()
        next_id = 0
        for model in world.findall('model'):
            name = model.get('name')
            parts = []
            for link in model.findall('link'):
                link_m = pose_matrix(link.findtext('pose'))
                for visual in link.findall('visual'):
                    marker = self.marker_for(visual.find('geometry'))
                    if marker is None:
                        continue
                    marker.ns, marker.id = name, next_id
                    next_id += 1
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color_of(visual)
                    local_m = link_m @ pose_matrix(visual.findtext('pose'))
                    if visual.find('geometry/plane') is not None:
                        # Slab top just below z=0 so it doesn't z-fight RViz's grid.
                        local_m = local_m @ pose_matrix(f'0 0 {-(FLOOR_THICKNESS / 2 + 0.001)}')
                    parts.append((marker, local_m))
            if not parts:
                continue
            self.models[name] = pose_matrix(model.findtext('pose')), parts
            if model.findtext('static', 'false').strip() not in ('true', '1'):
                self.dynamic.add(name)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'world_markers', latched)
        if self.dynamic:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, f'/world/{gz_world}/dynamic_pose/info', self.on_poses)
            self.create_timer(1.0 / rate, self.publish)
        self.publish()
        self.get_logger().info(
            f"{sum(len(p) for _, p in self.models.values())} markers from {len(self.models)} models "
            f"in '{gz_world}' ({len(self.dynamic)} following gz-sim) -> {self.frame_id}")

    def marker_for(self, geometry):
        shape = geometry[0] if geometry is not None and len(geometry) else None
        if shape is None:
            return None
        m = Marker()
        m.action = Marker.ADD
        if shape.tag == 'plane':
            sx, sy = (float(x) for x in shape.findtext('size', '0 0').split())
            if not 0 < max(sx, sy) <= MAX_FLOOR_SIZE:
                return None
            m.type = Marker.CUBE
            m.scale.x, m.scale.y, m.scale.z = sx, sy, FLOOR_THICKNESS
        elif shape.tag == 'box':
            m.type = Marker.CUBE
            m.scale.x, m.scale.y, m.scale.z = (float(x) for x in shape.findtext('size').split())
        elif shape.tag == 'cylinder':
            m.type = Marker.CYLINDER
            m.scale.x = m.scale.y = 2 * float(shape.findtext('radius'))
            m.scale.z = float(shape.findtext('length'))
        elif shape.tag == 'sphere':
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = 2 * float(shape.findtext('radius'))
        elif shape.tag == 'mesh':
            uri = shape.findtext('uri', '')
            if not uri.startswith(('file://', 'package://')):
                self.get_logger().warn(f"skipping mesh '{uri}': only file:// and package:// resolve in RViz")
                return None
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = uri
            m.mesh_use_embedded_materials = True
            m.scale.x, m.scale.y, m.scale.z = (float(x) for x in shape.findtext('scale', '1 1 1').split())
        else:
            self.get_logger().warn(f'skipping unsupported geometry <{shape.tag}>')
            return None
        return m

    def on_poses(self, msg):
        """gz-transport callback (its own thread): latest world pose of each moving model."""
        with self.lock:
            for p in msg.pose:
                if p.name in self.dynamic:
                    q = p.orientation
                    m = quaternion_matrix([q.x, q.y, q.z, q.w])
                    m[:3, 3] = [p.position.x, p.position.y, p.position.z]
                    self.models[p.name] = m, self.models[p.name][1]

    def publish(self):
        out = MarkerArray()
        with self.lock:
            for model_m, parts in self.models.values():
                for marker, local_m in parts:
                    world_m = model_m @ local_m
                    # Stamp 0: RViz uses the latest TF instead of waiting on sim-time lookups.
                    marker.header.frame_id = self.frame_id
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = world_m[:3, 3]
                    q = quaternion_from_matrix(world_m)
                    (marker.pose.orientation.x, marker.pose.orientation.y,
                     marker.pose.orientation.z, marker.pose.orientation.w) = q
                    out.markers.append(marker)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = WorldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
