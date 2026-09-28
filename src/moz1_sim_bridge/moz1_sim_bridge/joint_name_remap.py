#!/usr/bin/env python3
"""
joint_name_remap  --  bridge sim joint names <-> canonical URDF names.

The Isaac Sim joint I/O graph publishes/consumes JointState messages that carry
the SIM joint names (e.g. 'Base_0'), while the real-robot MoveIt stack speaks
canonical URDF names (e.g. 'Base-0', 'LeftArm-0'). This node sits on the host
and relays both directions so MoveIt's topic_based_ros2_control and the rest of
the stack stay byte-identical between sim and real:

  STATE   : sub  in_state_topic  (sim names)       -> pub out_state_topic  (canonical)
  COMMAND : sub  in_command_topic (canonical)      -> pub out_command_topic (sim names)

The mapping comes from a YAML file (see config/joint_name_map.yaml), produced by
the REMAP SKELETON that discover_prims.py prints. If the sim already uses
canonical names, an empty / identity map makes this a transparent pass-through.

Default wiring (matches attach_joint_io.py + topic_based_ros2_control config):
  in_state_topic   = /isaac/joint_states     (sim names, from Isaac)
  out_state_topic  = /joint_states           (canonical; MoveIt + RSP read this)
  in_command_topic = /joint_command          (canonical; from topic_based HW)
  out_command_topic= /isaac/joint_command    (sim names; Isaac applies this)

Names not present in the map pass through unchanged (so wheels/extra joints are
not dropped). Set ~drop_unmapped:=true to instead drop unmapped joints.
"""

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointNameRemap(Node):
    def __init__(self):
        super().__init__("joint_name_remap")

        self.declare_parameter("map_file", "")
        self.declare_parameter("in_state_topic", "/isaac/joint_states")
        self.declare_parameter("out_state_topic", "/joint_states")
        self.declare_parameter("in_command_topic", "/joint_command")
        self.declare_parameter("out_command_topic", "/isaac/joint_command")
        self.declare_parameter("drop_unmapped", False)

        gp = self.get_parameter
        map_file = gp("map_file").get_parameter_value().string_value
        self.drop_unmapped = gp("drop_unmapped").get_parameter_value().bool_value

        self.sim_to_canonical = self._load_map(map_file)
        self.canonical_to_sim = {v: k for k, v in self.sim_to_canonical.items()}
        self.get_logger().info(
            f"Loaded {len(self.sim_to_canonical)} joint name mappings "
            f"(drop_unmapped={self.drop_unmapped})"
        )

        in_state = gp("in_state_topic").get_parameter_value().string_value
        out_state = gp("out_state_topic").get_parameter_value().string_value
        in_cmd = gp("in_command_topic").get_parameter_value().string_value
        out_cmd = gp("out_command_topic").get_parameter_value().string_value

        self._pub_state = self.create_publisher(JointState, out_state, 10)
        self._pub_cmd = self.create_publisher(JointState, out_cmd, 10)
        self.create_subscription(
            JointState, in_state,
            lambda m: self._relay(m, self.sim_to_canonical, self._pub_state), 10)
        self.create_subscription(
            JointState, in_cmd,
            lambda m: self._relay(m, self.canonical_to_sim, self._pub_cmd), 10)

        self.get_logger().info(f"STATE   {in_state} (sim) -> {out_state} (canonical)")
        self.get_logger().info(f"COMMAND {in_cmd} (canonical) -> {out_cmd} (sim)")

    def _load_map(self, path):
        if not path:
            self.get_logger().warn("No map_file set -- running as identity pass-through.")
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except OSError as exc:
            self.get_logger().error(f"Could not read map_file {path}: {exc}")
            return {}
        mapping = data.get("sim_to_canonical", data)  # accept either layout
        # Drop blank/unfilled guesses so they pass through unchanged.
        return {k: v for k, v in mapping.items() if isinstance(v, str) and v}

    def _relay(self, msg, name_map, pub):
        out = JointState()
        out.header = msg.header

        n = len(msg.name)
        names, pos, vel, eff = [], [], [], []
        for i, src in enumerate(msg.name):
            dst = name_map.get(src)
            if dst is None:
                if self.drop_unmapped:
                    continue
                dst = src  # pass through
            names.append(dst)
            if i < len(msg.position):
                pos.append(msg.position[i])
            if i < len(msg.velocity):
                vel.append(msg.velocity[i])
            if i < len(msg.effort):
                eff.append(msg.effort[i])

        out.name = names
        # Keep arrays aligned: only emit position/velocity/effort if fully present.
        out.position = pos if len(pos) == len(names) else []
        out.velocity = vel if len(vel) == len(names) else []
        out.effort = eff if len(eff) == len(names) else []
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointNameRemap()
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
