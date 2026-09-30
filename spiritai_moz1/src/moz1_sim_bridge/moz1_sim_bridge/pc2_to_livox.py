#!/usr/bin/env python3
"""
pc2_to_livox  --  sensor_msgs/PointCloud2  ->  livox_ros_driver2/CustomMsg.

Isaac Sim's RTX lidar publishes a standard sensor_msgs/PointCloud2 on
/livox/lidar. The slam_toolbox path (pointcloud_to_laserscan) consumes that
directly. The FAST-LIO2 path, however, is configured for lidar_type=1, i.e. the
native livox_ros_driver2/CustomMsg, which Isaac cannot emit. This node converts
the sim PointCloud2 into CustomMsg so the UNCHANGED FAST-LIO2 config runs
against the simulator.

  SUB  in_topic   sensor_msgs/PointCloud2          (default /livox/lidar)
  PUB  out_topic  livox_ros_driver2/CustomMsg      (default /livox/lidar_custom)

Point the FAST-LIO2 config's lid_topic at out_topic (or remap), and set its
lidar_type to 1 (Livox). offset_time is synthesised by spreading points evenly
across the frame period (1 / scan_rate_hz); good enough for de-skew in sim.

Run only when exercising FAST-LIO2; the slam_toolbox path does not need it.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except ImportError as exc:  # pragma: no cover - depends on ws_livox overlay
    raise RuntimeError(
        "livox_ros_driver2 messages not found. Source your livox_ros_driver2 "
        "overlay (setup_env.sh) before running pc2_to_livox."
    ) from exc


class Pc2ToLivox(Node):
    def __init__(self):
        super().__init__("pc2_to_livox")
        self.declare_parameter("in_topic", "/livox/lidar")
        self.declare_parameter("out_topic", "/livox/lidar_custom")
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("lidar_id", 0)

        gp = self.get_parameter
        in_topic = gp("in_topic").get_parameter_value().string_value
        out_topic = gp("out_topic").get_parameter_value().string_value
        self.scan_period_ns = int(1e9 / max(1e-3,
                                  gp("scan_rate_hz").get_parameter_value().double_value))
        self.lidar_id = gp("lidar_id").get_parameter_value().integer_value

        self._pub = self.create_publisher(CustomMsg, out_topic, 10)
        self.create_subscription(PointCloud2, in_topic, self._cb, 10)
        self.get_logger().info(f"{in_topic} (PointCloud2) -> {out_topic} (CustomMsg)")

    def _cb(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        want = ["x", "y", "z"]
        has_intensity = "intensity" in field_names
        if has_intensity:
            want.append("intensity")

        pts = list(point_cloud2.read_points(
            msg, field_names=want, skip_nans=True))
        n = len(pts)
        if n == 0:
            return

        out = CustomMsg()
        out.header = msg.header
        # timebase: frame start in ns (CustomMsg uses its own uint64 timebase).
        out.timebase = int(msg.header.stamp.sec) * 1_000_000_000 + \
            int(msg.header.stamp.nanosec)
        out.point_num = n
        out.lidar_id = self.lidar_id
        out.rsvd = [0, 0, 0]

        step = self.scan_period_ns // n if n else 0
        custom_pts = []
        for i, p in enumerate(pts):
            cp = CustomPoint()
            cp.offset_time = int(i * step)
            cp.x = float(p[0])
            cp.y = float(p[1])
            cp.z = float(p[2])
            refl = float(p[3]) if has_intensity else 0.0
            cp.reflectivity = max(0, min(255, int(refl)))
            cp.tag = 0
            cp.line = 0
            custom_pts.append(cp)
        out.points = custom_pts
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Pc2ToLivox()
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
