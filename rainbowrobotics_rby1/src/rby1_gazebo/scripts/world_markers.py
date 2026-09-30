#!/usr/bin/env python3
"""Mirror the Gazebo world's models (table, objects, ...) into RViz as a MarkerArray.

RViz only shows what is on ROS topics, so with the Gazebo GUI off the world's
objects would be invisible. This node parses the same SDF file gz-sim loaded and
publishes one marker per box/cylinder/sphere <visual>, colored from its <material>.
Static models are drawn at their SDF pose; non-static ones (the cube, the cylinder)
follow the live simulation via gz-sim's /world/<name>/dynamic_pose/info.
Ground planes are skipped (RViz's grid stands in for the floor).

Markers are in the Gazebo world frame (`frame_id`, default "world"); the launch
file publishes world -> odom from the robot's spawn pose.
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


def marker_for(geometry):
    m = Marker()
    m.action = Marker.ADD
    if geometry.find('box') is not None:
        m.type = Marker.CUBE
        m.scale.x, m.scale.y, m.scale.z = (float(s) for s in geometry.findtext('box/size').split())
    elif geometry.find('cylinder') is not None:
        m.type = Marker.CYLINDER
        r = float(geometry.findtext('cylinder/radius'))
        m.scale.x = m.scale.y = 2 * r
        m.scale.z = float(geometry.findtext('cylinder/length'))
    elif geometry.find('sphere') is not None:
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 2 * float(geometry.findtext('sphere/radius'))
    else:
        return None  # planes, meshes: not drawn
    return m


class WorldMarkers(Node):
    def __init__(self):
        super().__init__('world_markers')
        world_file = self.declare_parameter('world_file', '').value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        ignore = set(self.declare_parameter('ignore_models', ['rby1']).value)

        world = ET.parse(world_file).getroot().find('world')
        # model name -> [world pose 4x4, [(marker, pose in model frame)]]
        self.models = {}
        self.lock = threading.Lock()
        dynamic = []
        next_id = 0
        for model in world.findall('model'):
            name = model.get('name')
            if name in ignore:
                continue
            parts = []
            for link in model.findall('link'):
                link_m = pose_matrix(link.findtext('pose'))
                for visual in link.findall('visual'):
                    marker = marker_for(visual.find('geometry'))
                    if marker is None:
                        continue
                    marker.header.frame_id = self.frame_id
                    marker.ns, marker.id = name, next_id
                    next_id += 1
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color_of(visual)
                    parts.append((marker, link_m @ pose_matrix(visual.findtext('pose'))))
            if parts:
                self.models[name] = [pose_matrix(model.findtext('pose')), parts]
                if model.findtext('static', 'false').strip() not in ('true', '1'):
                    dynamic.append(name)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'world_markers', latched)
        if dynamic:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, f"/world/{world.get('name')}/dynamic_pose/info", self.on_poses)
        self.create_timer(0.1, self.publish)
        self.get_logger().info(
            f'{next_id} markers from {len(self.models)} models ({len(dynamic)} following gz-sim)')

    def on_poses(self, msg):
        with self.lock:
            for p in msg.pose:
                if p.name in self.models:
                    m = quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
                    m[:3, 3] = [p.position.x, p.position.y, p.position.z]
                    self.models[p.name][0] = m

    def publish(self):
        out = MarkerArray()
        with self.lock:
            for model_m, parts in self.models.values():
                for marker, local_m in parts:
                    m = model_m @ local_m
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = m[:3, 3]
                    q = quaternion_from_matrix(m)
                    marker.pose.orientation.x, marker.pose.orientation.y = q[0], q[1]
                    marker.pose.orientation.z, marker.pose.orientation.w = q[2], q[3]
                    out.markers.append(marker)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(WorldMarkers())


if __name__ == '__main__':
    main()
