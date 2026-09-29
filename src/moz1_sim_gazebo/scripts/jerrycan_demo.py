#!/usr/bin/env python3
"""jerrycan_demo.py — Moz1 depalletises jerry cans onto the infeed conveyors.

Reads the scene YAML written by jerrycan_scene.py and, for each target can on the
pallet's top-front row (alternating arms), runs:

    open gripper → plan (OMPL) to 12 cm above the handle, jaw across the bar
    → straight down (Cartesian) → close gripper (grasp_helper welds the can)
    → straight up clear of the stack → plan to above a conveyor slot
    → straight down → open (released) → straight up

The can's pose is read from Gazebo before each pick (standing in for perception:
grasp where it IS, skip one that has been knocked over), and each place is
VERIFIED against Gazebo's own model pose (a one-shot read of /world/<world>/pose/
info via the `ign topic` CLI — ros_gz_bridge drops the model names from that
message): a can only counts as done if it really stands on its conveyor slot.

Everything goes through move_group's standard interfaces — the /move_action and
/execute_trajectory actions, /compute_cartesian_path and /apply_planning_scene —
plus the grippers' FollowJointTrajectory controllers, i.e. the same interfaces the
real robot exposes. Sim-specific: /grasp/<side>/target (String) tells
grasp_helper which can the next close should grab, /grasp/<side>/{attach,detach}
make that explicit, and the Gazebo pose reads above.

Poses are expressed in base_link. MoveIt's `world` is the SRDF virtual joint
(identity to base_link), while the gz world has base_link at (0, 0, base_z), so the
scene's world coordinates are shifted down by base_z.

Parameters:
  scene_file   path to the scene YAML (required)
  arms         "both" | "left" | "right"   which arm(s) to use (default both)
  count        max cans to move, 0 = all targets (default 0)
  speed        velocity scaling for free-space moves, 0..1 (default 0.4)
"""
import math
import re
import subprocess
import threading
import time

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ConfigureController, ListControllers, SwitchController
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject,
                             Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

FRAME = "base_link"
# Top-down grasp, jaw closing along base x (across the handle bar, which runs along
# y). TCP +Z is the approach axis and the fingers close along TCP ±Y, so the TCP
# frame is x_tcp = +y, y_tcp = +x, z_tcp = -z: a half turn about (1, 1, 0)/√2.
# The second entry is the same grasp yawed 180°, tried when the first won't plan.
GRASP_QUATS = [Quaternion(x=math.sqrt(0.5), y=math.sqrt(0.5), z=0.0, w=0.0),
               Quaternion(x=math.sqrt(0.5), y=-math.sqrt(0.5), z=0.0, w=0.0)]
# SDK stroke units (see the gripper macro and gripper_cmd_to_gap.csv; 0.10 ≈ 77 mm
# jaw gap, 0.08 ≈ 63 mm, 0.026 ≈ 26 mm). Release at 0.10, not the 0.12 end stop:
# at 0.12 the finger mimics are driven past their own joint limits. Approach
# half-open: a wide opening sweeps the jaw into the next row's handles. Close only
# to the 24 mm bar; grasp_helper grabs at < 0.04, before the jaw touches it.
GRIPPER_OPEN, GRIPPER_APPROACH, GRIPPER_CLOSED = 0.10, 0.08, 0.026
PRE_GRASP = 0.12          # approach / retreat height above the handle
LIFT_CLEAR = 0.08         # can bottom above the stack top when carried
HOME = {
    "left": [-0.1570796327, -0.872664626, -0.3490658504, -1.5707963268,
             -0.6108652381, 0.1396263402, -0.1221730476],
    "right": [0.1570796327, -0.872664626, 0.3490658504, 1.5707963268,
              0.6108652381, 0.1396263402, 0.1221730476],
}
GRIPPER_LINKS = ["gripper_base_link", "gripper_actuator_link", "gripper_tcp_link",
                 "gripper_narrow1_link", "gripper_narrow2_link", "gripper_narrow3_link",
                 "gripper_narrow4_link", "gripper_narrow_loop_link", "gripper_wide1_link",
                 "gripper_wide2_link", "gripper_wide3_link", "gripper_wide4_link",
                 "gripper_wide_loop_link"]


class StepFailed(RuntimeError):
    pass


def _pose(x, y, z, q):
    return Pose(position=Point(x=float(x), y=float(y), z=float(z)), orientation=q)


def _box(name, center, size, op=CollisionObject.ADD):
    co = CollisionObject(id=name, operation=op)
    co.header.frame_id = FRAME
    if op == CollisionObject.ADD:
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=[float(s) for s in size])]
        co.primitive_poses = [_pose(*center, Quaternion(w=1.0))]
    return co


class JerrycanDemo(Node):
    def __init__(self):
        super().__init__("jerrycan_demo")
        scene_file = self.declare_parameter("scene_file", "").value
        self.arms = self.declare_parameter("arms", "both").value
        self.count = int(self.declare_parameter("count", 0).value)
        self.speed = float(self.declare_parameter("speed", 0.4).value)
        with open(scene_file) as f:
            self.scene = yaml.safe_load(f)
        self.bz = self.scene["base_z"]

        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.grippers = {s: ActionClient(self, FollowJointTrajectory,
                                         f"/{s}_gripper_controller/follow_joint_trajectory")
                         for s in ("left", "right")}
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.apply_scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.list_controllers = self.create_client(ListControllers,
                                                   "/controller_manager/list_controllers")
        self.configure_controller = self.create_client(
            ConfigureController, "/controller_manager/configure_controller")
        self.switch_controller = self.create_client(
            SwitchController, "/controller_manager/switch_controller")
        self.target_pub = {s: self.create_publisher(String, f"/grasp/{s}/target", 10)
                           for s in ("left", "right")}
        self.weld_pub = {(s, a): self.create_publisher(Empty, f"/grasp/{s}/{a}", 10)
                         for s in ("left", "right") for a in ("attach", "detach")}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.joint_vel = {}
        self.create_subscription(
            JointState, "/joint_states",
            lambda m: self.joint_vel.update(zip(m.name, m.velocity)), 10)

    def settle(self, side, timeout=3.0):
        """Wait until the arm has stopped. The trajectory controller reports success
        when the trajectory's TIME is up, and a loaded arm is still catching up then;
        the next plan starts from where it is, and MoveIt rejects a trajectory whose
        start differs from the state at execution by > allowed_start_tolerance."""
        prefix = f"{side.capitalize()}Arm-"
        t0 = time.monotonic()
        time.sleep(0.2)
        while time.monotonic() - t0 < timeout:
            v = [abs(x) for n, x in self.joint_vel.items() if n.startswith(prefix)]
            if v and max(v) < 0.01:
                return
            time.sleep(0.1)
        self.get_logger().warn(f"{side} arm still moving after {timeout:.0f} s")

    def gz_model_positions(self):
        """{model name: (x, y, z)} for every model in the gz world, read straight
        from Gazebo. Empty dict if the `ign` CLI can't reach the sim."""
        topic = f"/world/{self.scene['world_name']}/pose/info"
        try:
            out = subprocess.run(["ign", "topic", "-e", "-t", topic, "-n", "1"],
                                 capture_output=True, text=True, timeout=15).stdout
        except (OSError, subprocess.TimeoutExpired):
            return {}
        found = {}
        for block in re.findall(r"pose \{(.*?)\n\}", out, re.S):
            name = re.search(r'name: "([^"]+)"', block)
            pos = re.search(r"position \{(.*?)\}", block, re.S)
            if name and pos:
                v = dict(re.findall(r"(\w): ([-\d.e+]+)", pos.group(1)))
                found[name.group(1)] = tuple(float(v.get(k, 0.0)) for k in "xyz")
        return found

    # ------------------------------------------------------------------ plumbing
    def _wait(self, future, timeout, what):
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout:
                raise StepFailed(f"timed out waiting for {what}")
            time.sleep(0.02)
        return future.result()

    def _send(self, client, goal, timeout, what):
        handle = self._wait(client.send_goal_async(goal), 10.0, f"{what} (accept)")
        if not handle.accepted:
            raise StepFailed(f"{what}: goal rejected")
        return self._wait(handle.get_result_async(), timeout, what).result

    def _w(self, x, y, z):
        """gz world → base_link."""
        return x, y, z - self.bz

    def wait_for_servers(self):
        self.get_logger().info("waiting for the controllers to activate ...")
        needed = {f"{s}_{k}_controller" for s in ("left", "right") for k in ("arm", "gripper")}
        t0 = time.monotonic()
        while True:
            if self.list_controllers.wait_for_service(timeout_sec=2.0):
                res = self._wait(self.list_controllers.call_async(ListControllers.Request()),
                                 10.0, "list_controllers")
                state = {c.name: c.state for c in res.controller}
                missing = sorted(n for n in needed if state.get(n) != "active")
                if not missing:
                    break
                self.get_logger().info(f"  ... not active yet: {missing}")
                if time.monotonic() - t0 > 30.0:
                    self._rescue_controllers(missing, state)
            time.sleep(3.0)
        self.get_logger().info("waiting for move_group ...")
        for c in [self.move, self.execute, *self.grippers.values()]:
            while not c.wait_for_server(timeout_sec=2.0):
                self.get_logger().info(f"  ... still waiting for {c._action_name}")
        for c in (self.cartesian, self.apply_scene):
            while not c.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"  ... still waiting for {c.srv_name}")

    def _rescue_controllers(self, names, state):
        """The spawner gives up if load_controller takes > 10 s on a busy machine:
        its retry is refused ("already loaded") and it exits, leaving the
        controller loaded but never activated. Finish the job for it."""
        for n in names:
            if state.get(n) == "unconfigured":
                self.get_logger().warn(f"{n}: loaded but never configured — configuring")
                self._wait(self.configure_controller.call_async(
                    ConfigureController.Request(name=n)), 20.0, "configure_controller")
        loaded = [n for n in names if n in state]
        if loaded:
            self.get_logger().warn(f"activating {loaded} (the spawner gave up)")
            req = SwitchController.Request(activate_controllers=loaded,
                                           strictness=SwitchController.Request.BEST_EFFORT,
                                           timeout=Duration(sec=10))
            self._wait(self.switch_controller.call_async(req), 30.0, "switch_controller")

    # ------------------------------------------------------------ planning scene
    def _apply(self, scene):
        req = ApplyPlanningScene.Request(scene=scene)
        if not self._wait(self.apply_scene.call_async(req), 10.0, "apply_planning_scene").success:
            raise StepFailed("apply_planning_scene failed")

    def _can_box(self, name, bottom, shrink=0.0, body_only=False):
        """A can as a MoveIt box: its full envelope (handle included) so carried cans
        are routed ABOVE the neighbours' handles, or just the body for the can in
        the hand (the handle is between the fingers)."""
        c, can = self.scene["container"], self.scene["can"]
        h = can["body_h"] if body_only else c["h"]
        x, y, z = self._w(*bottom)
        return _box(name, (x, y, z + h / 2 + shrink / 2),
                    (c["W"] - shrink, c["L"] - shrink, h - shrink))

    def setup_scene(self):
        ps = PlanningScene(is_diff=True)
        for b in self.scene["collision_boxes"]:
            ps.world.collision_objects.append(_box(b["name"], self._w(*b["center"]), b["size"]))
        for t in self.scene["targets"]:
            ps.world.collision_objects.append(self._can_box(t["name"], t["pos"]))
        self._apply(ps)

    def attach(self, side, name, bottom):
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        # 2 cm smaller and 1 cm higher than the can: it must not start in contact
        # with the layer it is lifted off, or with its neighbours 4 mm away.
        aco = AttachedCollisionObject(link_name=f"{side}_gripper_tcp_link",
                                      object=self._can_box(name, bottom, shrink=0.02,
                                                           body_only=True),
                                      touch_links=[f"{side}_{l}" for l in GRIPPER_LINKS])
        ps.robot_state.attached_collision_objects = [aco]
        self._apply(ps)

    def detach(self, side, name):
        """Drop the can from the hand AND from the world. MoveIt's detach puts the
        object back into the world where it is, i.e. around the still-open jaw, and
        a start state in collision makes the retreat (and, since OMPL validates the
        whole robot, every later plan for either arm) fail. It is re-added with
        add_can() once the hand has retreated."""
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name=f"{side}_gripper_tcp_link")
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        ps.robot_state.attached_collision_objects = [aco]
        ps.world.collision_objects = [_box(name, None, None, op=CollisionObject.REMOVE)]
        self._apply(ps)

    def add_can(self, name, bottom):
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [self._can_box(name, bottom)]
        self._apply(ps)

    def remove(self, name):
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [_box(name, None, None, op=CollisionObject.REMOVE)]
        self._apply(ps)

    # ------------------------------------------------------------------- motion
    def _move_group(self, group, constraints, speed):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = group
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = speed
        r.max_acceleration_scaling_factor = speed
        r.start_state.is_diff = True
        r.workspace_parameters.header.frame_id = FRAME
        r.workspace_parameters.min_corner.x = r.workspace_parameters.min_corner.y = -2.0
        r.workspace_parameters.min_corner.z = -1.0
        r.workspace_parameters.max_corner.x = r.workspace_parameters.max_corner.y = 2.0
        r.workspace_parameters.max_corner.z = 2.0
        r.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        res = self._send(self.move, goal, 60.0, f"{group} move")
        self.settle(group.split("_")[0])
        return res.error_code.val

    def plan_to_pose(self, side, pose):
        """OMPL to a TCP pose: the pose's own orientation first, then the other
        top-down grasp yaw. Returns the orientation it used — later straight-line
        moves must use the same one, or they'd need a half-turn of the wrist."""
        link = f"{side}_gripper_tcp_link"
        first = min(GRASP_QUATS, key=lambda g: -abs(g.x * pose.orientation.x + g.y *
                                                   pose.orientation.y + g.z * pose.orientation.z
                                                   + g.w * pose.orientation.w))
        for q in ([first] + [g for g in GRASP_QUATS if g is not first]) * 2:
            c = Constraints()
            pc = PositionConstraint(link_name=link, weight=1.0)
            pc.header.frame_id = FRAME
            pc.constraint_region = BoundingVolume(
                primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])],
                primitive_poses=[_pose(pose.position.x, pose.position.y, pose.position.z,
                                       Quaternion(w=1.0))])
            oc = OrientationConstraint(link_name=link, orientation=q, weight=1.0,
                                       absolute_x_axis_tolerance=0.03,
                                       absolute_y_axis_tolerance=0.03,
                                       absolute_z_axis_tolerance=0.08)
            oc.header.frame_id = FRAME
            c.position_constraints, c.orientation_constraints = [pc], [oc]
            code = self._move_group(f"{side}_arm", c, self.speed)
            if code == MoveItErrorCodes.SUCCESS:
                return q
            self.get_logger().warn(f"{side}_arm: plan to pose failed (code {code}), "
                                   f"trying the other grasp yaw")
        raise StepFailed(f"{side}_arm: no plan to {pose.position}")

    def go_home(self, side):
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=f"{side.capitalize()}Arm-{i}", position=v,
                            tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for i, v in enumerate(HOME[side])]
        code = self._move_group(f"{side}_arm", c, self.speed)
        if code != MoveItErrorCodes.SUCCESS:
            raise StepFailed(f"{side}_arm: home failed (code {code})")

    def straight(self, side, waypoints, speed=0.15):
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state.is_diff = True
        req.group_name = f"{side}_arm"
        req.link_name = f"{side}_gripper_tcp_link"
        req.waypoints = waypoints
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed
        res = self._wait(self.cartesian.call_async(req), 20.0, "compute_cartesian_path")
        if res.fraction < 0.98:
            raise StepFailed(f"{side}_arm: Cartesian path only {res.fraction:.0%} feasible")
        goal = ExecuteTrajectory.Goal(trajectory=res.solution)
        code = self._send(self.execute, goal, 60.0, f"{side}_arm straight").error_code.val
        self.settle(side)
        if code != MoveItErrorCodes.SUCCESS:
            raise StepFailed(f"{side}_arm: Cartesian execution failed (code {code})")

    def weld(self, side, act):
        """Ask grasp_helper to attach/detach explicitly. It also does this on the
        gripper's close/open edge, but the trajectory controller reports success
        when time runs out, reached or not, so don't rely on the edge alone. Both
        paths are idempotent."""
        self.weld_pub[(side, act)].publish(Empty())
        time.sleep(0.3)

    def gripper(self, side, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"{side}_gripper_joint"]
        pt = JointTrajectoryPoint(positions=[position])
        pt.time_from_start.sec = 1
        pt.time_from_start.nanosec = 500_000_000
        goal.trajectory.points = [pt]
        self._send(self.grippers[side], goal, 15.0, f"{side} gripper")

    def check_hand(self, side, pose, what):
        """Log how far the ACTUAL tcp (TF from Gazebo's joint states) is from where
        the plan put it. Returns the error in metres (None if TF is unavailable)."""
        time.sleep(0.3)
        try:
            t = self.tf.lookup_transform(FRAME, f"{side}_gripper_tcp_link", rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 — tf2 raises several types
            self.get_logger().warn(f"{side} {what}: no TF for the hand ({exc})")
            return None
        a, p = t.transform.translation, pose.position
        err = math.dist((a.x, a.y, a.z), (p.x, p.y, p.z))
        msg = (f"{side} {what}: hand at ({a.x:+.3f}, {a.y:+.3f}, {a.z:+.3f}), "
               f"{err * 1000:.0f} mm from target")
        # separate call sites: rclpy refuses one call site logging at two severities
        if err > 0.01:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)
        return err

    def hand_world(self, side):
        """Actual tcp position in the gz world frame (TF + base_z), or None."""
        try:
            t = self.tf.lookup_transform(FRAME, f"{side}_gripper_tcp_link", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        a = t.transform.translation
        return (a.x, a.y, a.z + self.bz)

    def real_time_factor(self):
        try:
            out = subprocess.run(["ign", "topic", "-e", "-t", "/stats", "-n", "1"],
                                 capture_output=True, text=True, timeout=15).stdout
            return float(re.search(r"real_time_factor: ([\d.e+-]+)", out).group(1))
        except (OSError, subprocess.TimeoutExpired, AttributeError, ValueError):
            return None

    def verify_placed(self, name, slot):
        """True if Gazebo has the can standing on its slot (±4 cm, belt height)."""
        time.sleep(1.0)
        got = self.gz_model_positions().get(name)
        if got is None:
            self.get_logger().warn(f"{name}: could not read its pose from Gazebo")
            return False
        dxy = math.hypot(got[0] - slot[0], got[1] - slot[1])
        ok = dxy < 0.04 and abs(got[2] - slot[2]) < 0.02
        msg = (f"{name}: Gazebo has it at ({got[0]:+.3f}, {got[1]:+.3f}, {got[2]:+.3f}) — "
               f"{'on' if ok else 'NOT on'} its conveyor slot ({dxy * 1000:.0f} mm off)")
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        return ok

    # --------------------------------------------------------------- the task
    def pick_and_place(self, target, slot):
        side, name = target["side"], target["name"]
        can = self.scene["can"]
        # Locate the can: Gazebo's ground truth stands in for perception here. Grasp
        # where it IS; a can that has been knocked out of place is not attempted.
        pos = self.gz_model_positions().get(name)
        if pos is None:
            self.get_logger().warn(f"{name}: no pose from Gazebo, using the scene layout")
            pos = target["pos"]
        elif math.dist(pos, target["pos"]) > 0.03:
            raise StepFailed(f"it is {math.dist(pos, target['pos']) * 1000:.0f} mm from where "
                             f"the pallet layout puts it (knocked over?)")
        gx, gy, gz = self._w(pos[0], pos[1] + can["handle_y"], pos[2] + can["grasp_z"])
        # place 3 mm above the belt so the can settles instead of starting in contact
        place_bottom = [slot[0], slot[1], slot[2] + 0.003]
        px, py, pz = self._w(place_bottom[0], place_bottom[1] + can["handle_y"],
                             place_bottom[2] + can["grasp_z"])
        lift = self.scene["stack_top"] - pos[2] + LIFT_CLEAR   # can bottom clears the stack
        carry_z = gz + lift

        self.get_logger().info(f"── {name} ({side} arm)")
        self.gripper(side, GRIPPER_APPROACH)
        q = self.plan_to_pose(side, _pose(gx, gy, gz + PRE_GRASP, GRASP_QUATS[0]))
        self.remove(name)                  # the fingers straddle it on the way down
        self.straight(side, [_pose(gx, gy, gz, q)])
        self.check_hand(side, _pose(gx, gy, gz, q), "at the handle")
        self.target_pub[side].publish(String(data=name))
        time.sleep(0.3)
        self.gripper(side, GRIPPER_CLOSED)
        self.check_hand(side, _pose(gx, gy, gz, q), "after closing")
        self.weld(side, "attach")
        self.attach(side, name, pos)
        self.straight(side, [_pose(gx, gy, gz + 0.05, q)], speed=0.1)
        held = self.gz_model_positions().get(name)
        if held is not None and held[2] < pos[2] + 0.03:
            raise StepFailed("the can did not come up with the hand (weld not made)")
        self.straight(side, [_pose(gx, gy, carry_z, q)], speed=0.2)
        q = self.plan_to_pose(side, _pose(px, py, max(pz + 0.08, carry_z - 0.1), q))
        self.straight(side, [_pose(px, py, pz, q)])
        self.check_hand(side, _pose(px, py, pz, q), "at the place slot")
        self.gripper(side, GRIPPER_OPEN)
        self.weld(side, "detach")
        self.detach(side, name)
        self.straight(side, [_pose(px, py, pz + PRE_GRASP, q)], speed=0.3)
        self.add_can(name, place_bottom)
        if not self.verify_placed(name, slot):
            raise StepFailed("the can did not arrive on the conveyor")

    def run(self):
        self.wait_for_servers()
        s = self.scene
        c = s["container"]
        self.get_logger().info(
            f"{c['label']} jerry cans, {s['mass']:.2f} kg filled, "
            f"{len(s['targets'])} targets on a {s['layers']}-layer stack")
        if s["mass"] > s["payload_per_arm"]:
            self.get_logger().warn(
                f"{s['mass']:.2f} kg exceeds the {s['payload_per_arm']:.0f} kg per-arm "
                f"PLACEHOLDER rating (Moz1's is unpublished): the sim will lift it, a real "
                f"arm may not. See the humanoid pallet-reach check.")
        self.setup_scene()
        sides = ("left", "right") if self.arms == "both" else (self.arms,)
        targets = [t for t in s["targets"] if t["side"] in sides]
        if self.count > 0:
            targets = targets[:self.count]
        used = {"left": 0, "right": 0}
        done = 0
        for t in targets:
            side = t["side"]
            slot = s["place_slots"][side][used[side]]
            rtf = self.real_time_factor()
            if rtf is not None and rtf < 0.3:
                self.get_logger().warn(f"Gazebo is running at {rtf:.2f}x real time")
            try:
                self.pick_and_place(t, slot)
                used[side] += 1
                done += 1
            except StepFailed as exc:
                self.get_logger().error(f"{t['name']}: {exc} — releasing and skipping")
                try:
                    self.gripper(side, GRIPPER_OPEN)
                    self.weld(side, "detach")
                    self.detach(side, t["name"])     # no-op if it was never attached
                    self.go_home(side)
                except StepFailed as exc2:
                    self.get_logger().error(f"recovery failed: {exc2}")
        for side in sides:
            try:
                self.go_home(side)
            except StepFailed as exc:
                self.get_logger().error(str(exc))
        self.get_logger().info(f"done: {done}/{len(targets)} jerry cans verified on the "
                               f"conveyors")


def main():
    rclpy.init()
    node = JerrycanDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
