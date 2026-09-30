#!/usr/bin/env python3
"""transfer_demo.py — a dual-arm robot moves one pair of jerry cans through the filling line.

Reads the scene YAML written by transfer_scene.py and the robot profile
(robots/<robot>.yaml: joint names, torso poses, gripper, grasp orientation, …), and
runs, once:

  A  pick two EMPTY cans (0.2 kg) at once, one per arm, from pallet A: the top
     layer's first remaining row (nothing in front of it), the innermost pair both
     arms can reach — leaning the torso over the pallet if needed (the least lean
     that works; checked with collision-aware IK, torso included). The base stays
     in front of the pallet.
  →  drive to the conveyor's LOAD station, set both cans on the belt.
  →  run the conveyor: the cans travel to the UNLOAD station and are filled to
     4.2 kg on the way, and turn red (ConveyorBelt gz plugin).
  →  drive to the UNLOAD station, pick both FILLED cans at once, curl both arms
     in (cans close to the chest: the load's centre of gravity nearer the torso).
  B  drive to pallet B, crouch/lean the torso with the arms still curled, then
     reach out and place both into the open box on the deck.

Every placement is verified against Gazebo's own poses. Both arms move together:
free-space moves are one `dual_arm` MoveIt plan to per-arm IK solutions;
straight-line moves are planned per arm (MoveIt Cartesian path) and sent to the two
arm controllers at the same time. The base drives with /cmd_vel on /odom; the
MoveIt scene is re-expressed in base_link after every drive (MoveIt's `world` is
the base — the SRDF virtual joint is fixed).

Parameters: scene_file, robot_file (the profile), speed.
"""
import math
import os
import random
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes, PlanningScene,
                             RobotState)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Empty, String

# The jerry-can demo's MoveIt / Gazebo plumbing lives in moz1_sim_gazebo (installed
# as plain scripts, not a Python package), which this workspace overlays.
sys.path.insert(0, os.path.join(get_package_prefix("moz1_sim_gazebo"), "lib", "moz1_sim_gazebo"))
from jerrycan_demo import (FRAME, LIFT_CLEAR, PRE_GRASP, JerrycanDemo,  # noqa: E402
                           StepFailed, _box, _pose)
from geometry_msgs.msg import Quaternion  # noqa: E402
from moveit_msgs.msg import AttachedCollisionObject  # noqa: E402

SIDES = ("left", "right")
# Robot specifics — joint names, torso poses (degrees; "home" and "carry" are
# required), the pick / deck torso candidates, gripper positions and grasp
# orientation — come from the robot profile (robots/<robot>.yaml).
# Curling the loaded arms in before driving: the hands are pulled back toward the
# chest by the first of these that plans (straight line or free space, the held
# cans collision-checked against the body). Height and hand spacing are kept.
TUCK_IN = [0.20, 0.15, 0.10, 0.05]     # m, toward the robot
MAX_JUMP = 0.35     # rad per 5 mm straight-line step; more = an IK branch flip (see _jump)
# Total travel of any one joint along a straight-line move. Holding the top-down
# hand orientation along a line near full reach (backing out of pallet A's 5th
# row) takes the wrist through a singularity: a valid but huge, slow swing (2-4
# rad of wrist over 0.4 m, a 50 s trajectory). Such moves go joint-space instead.
MAX_TRAVEL = 1.5    # rad


class TransferDemo(JerrycanDemo):
    def __init__(self):
        super().__init__()
        with open(self.declare_parameter("robot_file", "").value) as f:
            r = yaml.safe_load(f)
        self.robot = r
        self.torso = r["torso_joints"]
        self.presets = r["torso_presets"]
        self.pick_torsos, self.deck_torsos = r["pick_torsos"], r["deck_torsos"]
        self.arm_joints, self.arm_limits = r["arm_joints"], r["arm_limits"]
        self.home = r["home_arms"]
        g = r["gripper"]
        self.g_open, self.g_approach, self.g_closed = g["open"], g["approach"], g["closed"]
        self.grasp_quats = [Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) for q in r["grasp_quats"]]
        self.tcp = {s: r["tcp_link"].format(side=s) for s in SIDES}
        self.get_logger().info(f"robot: {r['label']}")
        self.odom = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.arm_fjt = {s: ActionClient(self, FollowJointTrajectory,
                                        f"/{s}_arm_controller/follow_joint_trajectory")
                        for s in SIDES}
        self.conveyor_start = self.create_publisher(Empty, "/conveyor/start", 10)
        # direct joint-space moves (see _direct_joint), per controller
        self.fjt = dict(self.arm_fjt, torso=ActionClient(
            self, FollowJointTrajectory, f"/{r['torso_controller']}/follow_joint_trajectory"))
        self.recolor_pub = self.create_publisher(String, "/scene_markers/recolor", 10)
        self.conveyor_done = threading.Event()
        self.create_subscription(Empty, "/conveyor/done",
                                 lambda _m: self.conveyor_done.set(), 10)
        self.joints = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.joints.update(zip(m.name, m.position)), 10)
        # MoveIt world objects, kept in gz world coordinates so they can be
        # re-expressed in base_link whenever the base has moved:
        #   name -> ("box", center, size) | ("can", bottom, body_only)
        self.objects = {}

    def wait_for_servers(self):
        """The arm / gripper controllers and move_group (JerrycanDemo), then the torso
        controller as well, and the torso at rest: until it holds the torso, the
        drive's sag watch would take a still-moving torso as its reference."""
        super().wait_for_servers()
        name = self.robot["torso_controller"]
        from controller_manager_msgs.srv import ListControllers
        while True:
            res = self._wait(self.list_controllers.call_async(ListControllers.Request()),
                             10.0, "list_controllers")
            if any(c.name == name and c.state == "active" for c in res.controller):
                break
            self.get_logger().info(f"  ... not active yet: {name}")
            time.sleep(2.0)
        self.settle("torso", timeout=10.0)

    # ------------------------------------------------------------------- base
    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, p.z, yaw)

    def _w(self, x, y, z):
        """gz world (= odom) → base_link, at the base's current pose."""
        bx, by, bz, yaw = self.odom
        dx, dy = x - bx, y - by
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy, z - bz

    def drive_to(self, x, y, name, timeout=120.0, vmax=0.25, accel=0.3):
        """Holonomic drive on /odom to (x, y), heading +x. Two phases, repeated:
        translate (with heading correction) to within 1.5 cm, then turn in place
        to within 0.7° — the base yaws a little while it translates, and chasing
        both at once rarely has them in tolerance at the same instant. Done at
        2 cm / 1°. Everything after a drive is computed from the ACTUAL base pose
        (and the cans are located in Gazebo before each pick), so up to
        3 cm / 1.5° left over is accepted. (Tighter, 1 cm / 0.5°, the two phases
        could bounce off each other for all six rounds at creep speed: a 1.3 m
        drive once took 120 s.)

        Acceleration is limited (ACCEL): gz's base plugin applies the commanded
        velocity instantly, and those jerks spike the torso's joint torques when the
        arms hold a load out front.

        Torso watch: carrying the filled pair, the torso's position loop holds at
        rest but now and then starts to sag while the base moves, and if the base
        keeps going the sag runs away to the joint limit (the torso folds). So at
        the first 2.5° of sag the base stops, the torso is given time to settle
        back (it does, from a few degrees), and the drive resumes; only a sag that
        doesn't recover fails the step."""
        self.get_logger().info(f"driving to the {name} station (y = {y:.2f} m)")
        t0 = time.monotonic()
        torso0 = [self.joints.get(j, 0.0) for j in self.torso]
        cur = [0.0, 0.0]            # last commanded world-frame velocity
        step = accel * 0.05         # m/s² × the 50 ms cycle

        def errors():
            bx, by, _, yaw = self.odom
            return x - bx, y - by, math.atan2(math.sin(-yaw), math.cos(-yaw)), yaw

        def sag():
            return max(abs(self.joints.get(j, 0.0) - j0) for j, j0 in zip(self.torso, torso0))

        def send(vx_w, vy_w, wz, yaw):
            if sag() > math.radians(2.5):
                self.cmd_vel.publish(Twist())
                cur[0] = cur[1] = 0.0
                s0 = sag()
                t_stop = time.monotonic()
                while sag() > math.radians(0.5) and time.monotonic() - t_stop < 15.0:
                    self.cmd_vel.publish(Twist())
                    time.sleep(0.1)
                if sag() > math.radians(2.5):
                    moved = ", ".join(
                        f"{j} {math.degrees(self.joints.get(j, 0.0) - j0):+.1f}°"
                        for j, j0 in zip(self.torso, torso0)
                        if abs(self.joints.get(j, 0.0) - j0) > math.radians(0.5))
                    raise StepFailed(f"torso sagged {math.degrees(sag()):.1f}° under the load "
                                     f"and did not recover ({moved}) — stopped")
                self.get_logger().info(f"  torso sagged {math.degrees(s0):.1f}°: stopped, it "
                                       f"settled back in {time.monotonic() - t_stop:.1f} s, "
                                       f"continuing")
            for i, v in enumerate((vx_w, vy_w)):
                cur[i] += max(-step, min(step, v - cur[i]))
            t = Twist()
            t.linear.x = math.cos(yaw) * cur[0] + math.sin(yaw) * cur[1]
            t.linear.y = -math.sin(yaw) * cur[0] + math.cos(yaw) * cur[1]
            t.angular.z = wz
            self.cmd_vel.publish(t)
            time.sleep(0.05)

        rounds = 0
        for rounds in range(1, 7):
            while True:                                   # translate
                ex, ey, eyaw, yaw = errors()
                d = math.hypot(ex, ey)
                if d < 0.015 or time.monotonic() - t0 > timeout:
                    break
                v = min(vmax, max(0.05, 1.2 * d))
                send(ex / d * v, ey / d * v, max(-0.3, min(0.3, 1.5 * eyaw)), yaw)
            while True:                                   # turn in place
                ex, ey, eyaw, yaw = errors()
                if abs(eyaw) < 0.012 or time.monotonic() - t0 > timeout:
                    break
                send(0.0, 0.0, math.copysign(max(0.04, min(0.3, 1.5 * abs(eyaw))), eyaw), yaw)
            for _ in range(20):                           # ramp down to rest
                send(0.0, 0.0, 0.0, yaw)
                if cur == [0.0, 0.0]:
                    break
            ex, ey, eyaw, _ = errors()
            if math.hypot(ex, ey) < 0.02 and abs(eyaw) < 0.017:
                break
        for _ in range(10):
            self.cmd_vel.publish(Twist())
            time.sleep(0.05)
        ex, ey, eyaw, yaw = errors()
        bx, by, _, _ = self.odom
        if math.hypot(ex, ey) > 0.03 or abs(eyaw) > 0.026:
            raise StepFailed(f"could not reach the {name} station: at ({bx:+.3f}, {by:+.3f}), "
                             f"heading {math.degrees(yaw):+.1f}°")
        self.get_logger().info(f"  at ({bx:+.3f}, {by:+.3f}), heading {math.degrees(yaw):+.2f}° "
                               f"({math.hypot(ex, ey) * 1000:.0f} mm off; {rounds} round(s), "
                               f"{time.monotonic() - t0:.1f} s)")
        self.refresh_scene()

    # ----------------------------------------------------------- planning scene
    def add_object(self, name, kind, a, b):
        self.objects[name] = (kind, a, b)
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [self._object(name)]
        self._apply(ps)

    def _object(self, name):
        kind, a, b = self.objects[name]
        if kind == "box":
            return _box(name, self._w(*a), b)
        return self._can_box(name, a, body_only=b)

    def remove(self, name):
        if self.objects.pop(name, None) is not None:
            super().remove(name)

    def attach(self, side, name, bottom):
        """The held can on the tcp link; it may touch the robot's gripper links."""
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        # 2 cm smaller and 1 cm higher than the can: it must not start in contact
        # with the layer it is lifted off, or with its neighbours 4 mm away.
        aco = AttachedCollisionObject(link_name=self.tcp[side],
                                      object=self._can_box(name, bottom, shrink=0.02,
                                                           body_only=True),
                                      touch_links=[f"{side}_{l}"
                                                   for l in self.robot["touch_links"]])
        ps.robot_state.attached_collision_objects = [aco]
        self._apply(ps)

    def detach(self, side, name):
        self.objects.pop(name, None)
        super().detach(side, name)

    def refresh_scene(self):
        """Re-express every world object in base_link after the base moved."""
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [self._object(n) for n in self.objects]
        self._apply(ps)

    def setup_scene(self):
        for b in self.scene["collision_boxes"]:
            self.objects[b["name"]] = ("box", b["center"], b["size"])
        # pick candidates as bodies only: the IK reachability check puts the
        # fingers around their handles
        for t in self.scene["targets"]:
            self.objects[t["name"]] = ("can", t["pos"], True)
        self.refresh_scene()

    # ----------------------------------------------------------------- motion
    def settle(self, side, timeout=3.0):
        a = self.arm_joints
        joints = set({"left": a["left"], "right": a["right"], "torso": self.torso,
                      "dual": a["left"] + a["right"]}.get(side, []))
        t0 = time.monotonic()
        time.sleep(0.2)
        while time.monotonic() - t0 < timeout:
            v = [abs(x) for n, x in self.joint_vel.items() if n in joints]
            if v and max(v) < 0.01:
                return
            time.sleep(0.1)
        self.get_logger().warn(f"{side} still moving after {timeout:.0f} s")

    def solve_ik(self, side, pose, torso=None, attempts=4, seed=None):
        """Collision-aware IK for one arm's tcp. `torso` (degrees) evaluates the
        pose as if the torso were there; `seed` (7 arm joints) starts the solver
        elsewhere than the current arm state. Returns the 7 joint values or None."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = f"{side}_arm"
        r.ik_link_name = self.tcp[side]
        r.avoid_collisions = True
        r.pose_stamped.header.frame_id = FRAME
        r.pose_stamped.pose = pose
        r.timeout = Duration(sec=0, nanosec=200_000_000)
        r.robot_state = RobotState(is_diff=True)
        names, values = [], []
        if torso is not None:
            names += self.torso
            values += [math.radians(d) for d in torso]
        if seed is not None:
            names += self.arm_joints[side]
            values += list(seed)
        r.robot_state.joint_state.name = names
        r.robot_state.joint_state.position = values
        for _ in range(attempts):
            res = self._wait(self.ik.call_async(req), 10.0, "compute_ik")
            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in self.arm_joints[side]]
        return None

    def dual_ik(self, poses, torso=None, prefer=None):
        """IK for both arms; tries the two top-down grasp yaws per arm, `prefer`
        ({side: quat}, the hand's current one) first — the other yaw is a
        half-turn of the wrist, and a joint move there twirls a held can.
        Returns ({side: joints}, {side: quat}) or None."""
        sols, quats = {}, {}
        for side in SIDES:
            order = self.grasp_quats
            if prefer and side in prefer:
                order = [prefer[side]] + [g for g in self.grasp_quats if g != prefer[side]]
            for q in order:
                p = poses[side]
                j = self.solve_ik(side, _pose(p[0], p[1], p[2], q), torso)
                if j is not None:
                    sols[side], quats[side] = j, q
                    break
            else:
                return None
        return sols, quats

    @staticmethod
    def _jump(traj):
        """Largest single-joint change between consecutive points (rad). The
        paths are sampled every 5 mm, so a big one is the IK flipping to another
        elbow/wrist branch mid-line — executed, that's the arm swinging wildly
        around (and then stuck, holding a can against something)."""
        pts = traj.points
        return max((abs(b - a) for p, q in zip(pts, pts[1:])
                    for a, b in zip(p.positions, q.positions)), default=0.0)

    @staticmethod
    def _travel(traj):
        """Largest total path length of any one joint along the trajectory (rad)."""
        pts = traj.points
        n = len(pts[0].positions) if pts else 0
        return max((sum(abs(q.positions[k] - p.positions[k]) for p, q in zip(pts, pts[1:]))
                    for k in range(n)), default=0.0)

    def _cartesian_ok(self, side, start_joints, pose):
        """Is a straight tcp line from `start_joints` to `pose` fully feasible?"""
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state = RobotState(is_diff=True)
        req.start_state.joint_state.name = self.arm_joints[side]
        req.start_state.joint_state.position = list(start_joints)
        req.group_name = f"{side}_arm"
        req.link_name = self.tcp[side]
        req.waypoints = [pose]
        req.max_step = 0.005
        # kinematics only: collisions are checked on the synchronized two-arm
        # motion when it runs (dual_straight), not with the other arm frozen
        req.avoid_collisions = False
        res = self._wait(self.cartesian.call_async(req), 20.0, "compute_cartesian_path")
        t = res.solution.joint_trajectory
        return res.fraction >= 0.98 and self._jump(t) < MAX_JUMP and self._travel(t) < MAX_TRAVEL

    def corridor_ik(self, side, anchor, reach, tries=12):
        """IK for `anchor` (x, y, z) from which straight vertical lines to each z
        in `reach` are feasible — e.g. down to the handle and up to the lift
        height. KDL returns one configuration per seed and a long vertical line
        can run one elbow configuration into a joint limit halfway, so sample
        seeds until a configuration covers the whole corridor.

        Seeds nearest first: the arm's current joints, then small random
        offsets from them, and only then anywhere in the joint range — a far
        seed tends to give a solution on the other elbow/wrist branch, which the
        arm then swings a long way round to reach."""
        lim = self.arm_limits[side]
        now = [self.joints.get(j, 0.0) for j in self.arm_joints[side]]
        for i in range(tries):
            if i == 0:
                seed = now
            elif i < tries // 2:
                seed = [min(b, max(a, v + random.gauss(0.0, 0.3 * i)))
                        for v, (a, b) in zip(now, lim)]
            else:
                seed = [random.uniform(a, b) for a, b in lim]
            for q in self.grasp_quats:
                j = self.solve_ik(side, _pose(*anchor, q), attempts=1, seed=seed)
                if j is not None and all(
                        self._cartesian_ok(side, j, _pose(anchor[0], anchor[1], z, q))
                        for z in reach):
                    return j, q
        return None

    def dual_corridor(self, anchors, reach, what):
        """Both arms to `anchors` in configurations that cover the vertical
        corridor `reach` (per side: list of z). Returns the quats used."""
        sols, quats = {}, {}
        for side in SIDES:
            found = self.corridor_ik(side, anchors[side], reach[side])
            if found is None:
                raise StepFailed(f"{side}_arm: no configuration covers the straight "
                                 f"moves {what}")
            sols[side], quats[side] = found
        self.dual_joint_goal(sols, what)
        return quats

    def _direct_joint(self, goals, vmax, samples=20):
        """Straight to `goals` ({controller: (joint names, positions)}) in joint
        space, if that line is collision-free with everything moving together:
        one smooth two-point trajectory per controller (the least joint motion
        there is), no planner detours. Returns False, having moved nothing, if
        the line collides. `vmax`: peak joint speed (rad/s)."""
        start = {k: [self.joints.get(j, 0.0) for j in names] for k, (names, _) in goals.items()}
        names = [j for k in goals for j in goals[k][0]]
        for i in range(samples + 1):
            f = i / samples
            req = GetStateValidity.Request(group_name="dual_arm")
            req.robot_state = RobotState(is_diff=True)
            req.robot_state.joint_state.name = names
            req.robot_state.joint_state.position = [
                a + (b - a) * f for k in goals for a, b in zip(start[k], goals[k][1])]
            res = self._wait(self.validity.call_async(req), 10.0, "check_state_validity")
            if not res.valid:
                c = res.contacts[0] if res.contacts else None
                big = max(abs(b - a) for k in goals for a, b in zip(start[k], goals[k][1]))
                self.get_logger().info(
                    f"  direct joint move ({math.degrees(big):.0f}° max) collides at {f:.0%}: "
                    f"{(c.contact_body_1 + ' / ' + c.contact_body_2) if c else 'invalid state'}")
                return False
        delta = max(abs(b - a) for k in goals for a, b in zip(start[k], goals[k][1]))
        dur = max(1.0, 1.5 * delta / vmax)      # cubic, zero end velocities: peak 1.5x mean
        handles = {}
        for k, (jn, goal) in goals.items():
            t = JointTrajectory(joint_names=jn)
            end = JointTrajectoryPoint(positions=list(goal), velocities=[0.0] * len(jn))
            end.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            t.points = [JointTrajectoryPoint(positions=start[k], velocities=[0.0] * len(jn)), end]
            handles[k] = self._wait(self.fjt[k].send_goal_async(
                FollowJointTrajectory.Goal(trajectory=t)), 10.0, f"{k} (accept)")
        for k, h in handles.items():
            if not h.accepted:
                raise StepFailed(f"{k}: trajectory rejected")
            res = self._wait(h.get_result_async(), dur + 30.0, k).result
            if res.error_code != 0:
                raise StepFailed(f"{k}: controller error {res.error_code}")
        return True

    def dual_joint_goal(self, sols, what):
        if self._direct_joint({sd: (self.arm_joints[sd], sols[sd]) for sd in SIDES}, self.speed):
            self.settle("dual")
            return
        self.get_logger().info(f"  planning {what} around the obstacle instead")
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=n, position=v, tolerance_above=0.005,
                            tolerance_below=0.005, weight=1.0)
            for side in SIDES for n, v in zip(self.arm_joints[side], sols[side])]
        code = self._move_group("dual_arm", c, self.speed)
        if code != MoveItErrorCodes.SUCCESS:
            raise StepFailed(f"dual_arm: no plan {what} (code {code})")

    def lean_to(self, name, arms):
        """Torso to preset `name` and both arms to `arms` (the pre-grasp joints
        for that torso) in ONE direct move. Leaning with the arms still at home
        drives the hands into the stack for the far rows; this goes straight to
        a pose the reach check already found collision-free. Falls back to torso
        first, then arms."""
        goals = {"torso": (self.torso, [math.radians(d) for d in self.presets[name]])}
        goals.update({sd: (self.arm_joints[sd], arms[sd]) for sd in SIDES})
        if self._direct_joint(goals, 0.5 * 0.3):
            self.settle("torso")
            self.settle("dual")
            self.get_logger().info(f"torso → {name}, hands over the pair")
            return
        self.torso_to(name)

    def torso_to(self, name):
        goal = [math.radians(d) for d in self.presets[name]]
        if self._direct_joint({"torso": (self.torso, goal)}, 0.5 * 0.3):
            self.settle("torso")
            self.get_logger().info(f"torso → {name}")
            return
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=n, position=math.radians(d), tolerance_above=0.005,
                            tolerance_below=0.005, weight=1.0)
            for n, d in zip(self.torso, self.presets[name])]
        code = self._move_group("torso", c, 0.3)
        if code != MoveItErrorCodes.SUCCESS:
            raise StepFailed(f"torso: no plan to '{name}' (code {code})")
        self.get_logger().info(f"torso → {name}")

    def _plan_straight(self, poses, quats, speed, avoid):
        trajs = {}
        for side in SIDES:
            req = GetCartesianPath.Request()
            req.header.frame_id = FRAME
            req.start_state.is_diff = True
            req.group_name = f"{side}_arm"
            req.link_name = self.tcp[side]
            p = poses[side]
            req.waypoints = [_pose(p[0], p[1], p[2], quats[side])]
            req.max_step = 0.005
            req.avoid_collisions = avoid
            req.max_velocity_scaling_factor = speed
            req.max_acceleration_scaling_factor = speed
            res = self._wait(self.cartesian.call_async(req), 20.0, "compute_cartesian_path")
            if res.fraction < 0.98:
                raise StepFailed(f"{side}_arm: straight move only {res.fraction:.0%} feasible")
            jump = self._jump(res.solution.joint_trajectory)
            if jump >= MAX_JUMP:
                raise StepFailed(f"{side}_arm: straight move flips configuration "
                                 f"({math.degrees(jump):.0f}° in one 5 mm step)")
            travel = self._travel(res.solution.joint_trajectory)
            if travel >= MAX_TRAVEL:
                raise StepFailed(f"{side}_arm: straight move swings a joint "
                                 f"{math.degrees(travel):.0f}° (wrist singularity)")
            trajs[side] = res.solution.joint_trajectory
        return trajs

    def _together_valid(self, trajs, samples=25):
        """Collision-check the two arms moving TOGETHER: both trajectories sampled at
        the same progress, each combined state checked for the whole robot."""
        names = trajs["left"].joint_names + trajs["right"].joint_names
        for i in range(samples + 1):
            f = i / samples
            pos = []
            for side in SIDES:
                pts = trajs[side].points
                pos += list(pts[round(f * (len(pts) - 1))].positions)
            req = GetStateValidity.Request(group_name="dual_arm")
            req.robot_state = RobotState(is_diff=True)
            req.robot_state.joint_state.name = names
            req.robot_state.joint_state.position = pos
            res = self._wait(self.validity.call_async(req), 10.0, "check_state_validity")
            if not res.valid:
                c = res.contacts[0] if res.contacts else None
                self.get_logger().warn(
                    f"moving together, {f:.0%} of the way: "
                    f"{(c.contact_body_1 + ' / ' + c.contact_body_2) if c else 'invalid state'}")
                return False
        return True

    def dual_straight(self, poses, quats, speed=0.15, check=True):
        """Straight-line tcp moves for both arms, executed on both arm controllers at
        once. Planned per arm with collision checking; if that fails only because
        each arm was checked against the other one standing still, the pair is
        re-checked moving together (the two hands hold neighbouring cans 18 cm
        apart, and a can rising past the other, still-low wrist is not what happens).
        check=False skips collision checking: only for the few-cm corrections of a
        guarded placement, which may start with a held can touching the box wall."""
        if not check:
            trajs = self._plan_straight(poses, quats, speed, avoid=False)
        else:
            try:
                trajs = self._plan_straight(poses, quats, speed, avoid=True)
            except StepFailed as exc:
                trajs = self._plan_straight(poses, quats, speed, avoid=False)
                if not self._together_valid(trajs):
                    raise exc
        handles = {}
        for side in SIDES:
            goal = FollowJointTrajectory.Goal(trajectory=trajs[side])
            goal.trajectory.header.stamp.sec = 0      # start now
            goal.trajectory.header.stamp.nanosec = 0
            handles[side] = self._wait(self.arm_fjt[side].send_goal_async(goal), 10.0,
                                       f"{side} arm (accept)")
        for side in SIDES:
            if not handles[side].accepted:
                raise StepFailed(f"{side} arm: trajectory rejected")
            res = self._wait(handles[side].get_result_async(), 60.0, f"{side} arm").result
            if res.error_code != 0:
                raise StepFailed(f"{side} arm: controller error {res.error_code}")
        self.settle("dual")

    def dual_move_to(self, poses, quats, what, speed=0.2):
        """Straight line if both arms can; otherwise IK + one free-space dual_arm
        plan (a long straight lift can run one arm into a joint limit on its
        current elbow configuration while another configuration reaches fine)."""
        try:
            self.dual_straight(poses, quats, speed)
            return quats
        except StepFailed as exc:
            self.get_logger().info(f"{exc} — planning {what} in free space instead")
        found = self.dual_ik(poses, prefer=quats)
        if found is None:
            raise StepFailed(f"no IK for {what}")
        sols, q = found
        self.dual_joint_goal(sols, what)
        return q

    def dual_gripper(self, position):
        goal_threads = [threading.Thread(target=self.gripper, args=(s, position)) for s in SIDES]
        for t in goal_threads:
            t.start()
        for t in goal_threads:
            t.join()

    def arms_home(self):
        self.dual_joint_goal(self.home, "home")

    def go_home(self):
        """Torso and both arms to the robot's home pose, together, in one direct
        joint move; the base stays where it is. If that line collides, the torso
        goes first, then the arms."""
        goals = {"torso": (self.torso, [math.radians(d) for d in self.presets["home"]])}
        goals.update({sd: (self.arm_joints[sd], self.home[sd]) for sd in SIDES})
        if self._direct_joint(goals, 0.5 * 0.3):
            self.settle("torso")
            self.settle("dual")
        else:
            self.torso_to("home")
            self.arms_home()
        self.get_logger().info("home: torso and arms at home (base stays)")

    # --------------------------------------------------------------- the task
    def grasp_pose(self, bottom):
        """base_link tcp position for a can whose bottom centre is at `bottom` (gz):
        the handle bar's centre, raised by the robot's grasp_z_offset (fingers that
        reach further below the tcp than the bar is tall grasp higher)."""
        can = self.scene["can"]
        return self._w(bottom[0], bottom[1] + can["handle_y"],
                       bottom[2] + can["grasp_z"] + self.robot["grasp_z_offset"])

    def locate(self, name, expected, tol=0.03):
        pos = self.gz_model_positions().get(name)
        if pos is None:
            raise StepFailed(f"{name}: no pose from Gazebo")
        if math.dist(pos, expected) > tol:
            raise StepFailed(f"{name} is {math.dist(pos, expected) * 1000:.0f} mm from where "
                             f"it should be")
        return pos

    def select_pick(self):
        """Top layer, first remaining row (nothing in front), innermost pair both
        arms reach, with the least torso lean. Returns ({side: target}, preset,
        {side: arm joints at the pre-grasp pose with that torso})."""
        by_side = {s: sorted([t for t in self.scene["targets"] if t["side"] == s],
                             key=lambda t: abs(t["pos"][1])) for s in SIDES}
        row = self.scene["cleared_rows"] + 1
        self.get_logger().info(
            f"pallet A: top layer rows 1-{row - 1} already taken, so row {row} is the "
            f"furthest row with nothing in front; checking reach (torso included) ...")
        for li, ri in sorted(((i, j) for i in range(len(by_side["left"]))
                              for j in range(len(by_side["right"]))), key=lambda p: max(p)):
            pair = {"left": by_side["left"][li], "right": by_side["right"][ri]}
            for preset in self.pick_torsos:
                torso = self.presets[preset]
                found = []
                for dz in (PRE_GRASP, 0.0):
                    poses = {s: tuple(v + (dz if k == 2 else 0) for k, v in
                                      enumerate(self.grasp_pose(pair[s]["pos"]))) for s in SIDES}
                    found.append(self.dual_ik(poses, torso))
                    if found[-1] is None:
                        break
                if found[-1] is not None:
                    self.get_logger().info(
                        f"picking {pair['left']['name']} (left) + {pair['right']['name']} "
                        f"(right), torso '{preset}'")
                    return pair, preset, found[0][0]
        raise StepFailed("no pair on pallet A is reachable by both arms")

    def pick_pair(self, names, bottoms, lift_to, where):
        """Descend, grab, lift both. `bottoms` are the cans' current gz positions."""
        grasp = {s: self.grasp_pose(bottoms[s]) for s in SIDES}
        pre = {s: (g[0], g[1], g[2] + PRE_GRASP) for s, g in grasp.items()}
        quats = self.dual_corridor(pre, {s: [grasp[s][2], grasp[s][2] + lift_to] for s in SIDES},
                                   f"above the {where} pair")
        for s in SIDES:
            self.remove(names[s])       # the fingers straddle the handles on the way down
        self.dual_straight(grasp, quats)
        for s in SIDES:
            self.target_pub[s].publish(String(data=names[s]))
        time.sleep(0.3)
        self.dual_gripper(self.g_closed)
        for s in SIDES:
            self.weld(s, "attach")
            self.attach(s, names[s], bottoms[s])
        self.dual_straight({s: (g[0], g[1], g[2] + 0.05) for s, g in grasp.items()}, quats, 0.1)
        now = self.gz_model_positions()
        for s in SIDES:
            if now.get(names[s], (0, 0, -1))[2] < bottoms[s][2] + 0.03:
                raise StepFailed(f"{names[s]} did not come up with the {s} hand")
        return self.dual_move_to({s: (g[0], g[1], g[2] + lift_to) for s, g in grasp.items()},
                                 quats, f"the lift off the {where} spot")

    def place_pair(self, names, slots, quats, what):
        """From above `slots` (can-bottom gz positions): guarded descent, release,
        up; verify.

        Guarded: loaded, the arms and torso sit a little lower than commanded
        (and more so crouched with 8.4 kg), and the held can goes wherever the hand
        is — so a descent to the nominal height pushed the can's bottom into the
        pallet deck. The descent stops 3 cm short, measures where each can's
        bottom really is (Gazebo), and corrects the rest by that error (twice).
        A sideways error over 5 mm (the can held off-centre, e.g. the hand lagging
        when it closed) is corrected too, so the can goes down over its slot."""
        gap = 0.003                             # can bottom above the surface at release
        place = {s: list(self.grasp_pose((slots[s][0], slots[s][1], slots[s][2] + gap)))
                 for s in SIDES}
        stand_off = 0.03
        target = {s: (p[0], p[1], p[2] + stand_off) for s, p in place.items()}
        yaw = self.odom[3]
        c, sn = math.cos(yaw), math.sin(yaw)
        for step in range(3):
            self.dual_straight(target, quats, 0.1 if step else 0.15, check=step == 0)
            now = self.gz_model_positions()
            want = stand_off if step == 0 else 0.0
            errs, side_err = {}, {}
            for s in SIDES:
                got = now.get(names[s])
                if got is None:
                    raise StepFailed(f"{names[s]}: no pose from Gazebo while placing")
                # + = the can sits higher than wanted, - = lower (would go into the surface)
                errs[s] = got[2] - (slots[s][2] + gap + want)
                dx, dy = got[0] - slots[s][0], got[1] - slots[s][1]     # gz world
                side_err[s] = (c * dx + sn * dy, -sn * dx + c * dy)      # base_link
            self.get_logger().info(
                f"placing on the {what}: can bottoms " + ", ".join(
                    f"{s} {errs[s] * 1000:+.0f} mm (x {side_err[s][0] * 1000:+.0f}, "
                    f"y {side_err[s][1] * 1000:+.0f})" for s in SIDES)
                + (" vs 3 cm above" if step == 0 else " vs the release height"))
            if step > 0 and all(abs(e) < 0.003 and math.hypot(*side_err[s]) <= 0.005
                                for s, e in errs.items()):
                break
            target = {s: (t[0] - (side_err[s][0] if math.hypot(*side_err[s]) > 0.005 else 0.0),
                          t[1] - (side_err[s][1] if math.hypot(*side_err[s]) > 0.005 else 0.0),
                          t[2] - want - errs[s]) for s, t in target.items()}
        self.dual_gripper(self.g_open)
        for s in SIDES:
            self.weld(s, "detach")
            self.detach(s, names[s])
        self.dual_straight({s: (t[0], t[1], t[2] + PRE_GRASP) for s, t in target.items()},
                           quats, 0.3)
        time.sleep(1.0)
        now = self.gz_model_positions()
        for s in SIDES:
            got = now.get(names[s])
            off = math.hypot(got[0] - slots[s][0], got[1] - slots[s][1]) if got else None
            if got is None or off > 0.04 or abs(got[2] - slots[s][2]) > 0.02:
                raise StepFailed(f"{names[s]} is not on its {what} slot (Gazebo: {got})")
            self.get_logger().info(f"{names[s]}: on the {what} ({off * 1000:.0f} mm off its slot)")
            self.add_object(names[s], "can", list(got), False)

    def tcp_now(self, side):
        t = self.tf.lookup_transform(FRAME, self.tcp[side], rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, t.transform.translation.z

    def above(self, slots, height, quats=None):
        """Both hands `height` above the handles of cans standing at `slots`."""
        poses = {s: tuple(v + (height if k == 2 else 0)
                          for k, v in enumerate(self.grasp_pose(slots[s]))) for s in SIDES}
        return self.dual_corridor(poses, {s: [poses[s][2] - height] for s in SIDES},
                                  "above the slots")

    def tuck(self, quats):
        """Curl both loaded arms in (see TUCK_IN). Best effort: if no tuck plans,
        carry as before. Returns the quats in use."""
        now = {s: self.tcp_now(s) for s in SIDES}
        for dx in TUCK_IN:
            try:
                q = self.dual_move_to({s: (p[0] - dx, p[1], p[2]) for s, p in now.items()},
                                      quats, f"the tuck ({dx * 100:.0f} cm in)", speed=0.1)
                self.get_logger().info(f"arms curled in {dx * 100:.0f} cm for the carry")
                return q
            except StepFailed as exc:
                self.get_logger().info(f"  tuck {dx * 100:.0f} cm: {exc}")
        self.get_logger().warn("no tuck planned: carrying with the arms out")
        return quats

    def run(self):
        self.wait_for_servers()
        for c in [self.ik]:
            while not c.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("  ... still waiting for /compute_ik")
        for c in self.arm_fjt.values():
            c.wait_for_server()
        while self.odom is None:
            self.get_logger().info("  ... waiting for /odom")
            time.sleep(1.0)
        s = self.scene
        st = s["stations"]
        off = s["belt_offset"]
        top = s["conveyor"]["top"]
        self.get_logger().info(
            f"4 L jerry cans: {s['empty_mass']} kg empty, {s['filled_mass']} kg filled. "
            f"One pair: pallet A → conveyor → filling → box on pallet B")
        self.drive_to(*st["A"], "pallet A")
        self.setup_scene()
        self.dual_gripper(self.g_approach)

        # --- pallet A: pick the empty pair
        pair, preset, pre_arms = self.select_pick()
        names = {s_: pair[s_]["name"] for s_ in SIDES}
        # the cans left behind: full height again (handles included), so nothing
        # carried is routed through them
        for t in s["targets"]:
            if t["name"] not in names.values():
                self.objects[t["name"]] = ("can", t["pos"], False)
        self.refresh_scene()
        bottoms = {s_: self.locate(names[s_], pair[s_]["pos"]) for s_ in SIDES}
        if preset != "home":
            self.lean_to(preset, pre_arms)
        # Lift just clear of the layer underneath, then slide straight back toward
        # the robot: the row's other cans are beside the pair (4 mm gaps, no
        # overlap across the row) and the rows in front are already empty. A high
        # lift over the neighbours would put the hands at the edge of their reach,
        # and a free-space path out of a tightly packed row clips the neighbours.
        quats = self.pick_pair(names, bottoms, LIFT_CLEAR, "pallet-A")
        self.get_logger().info("picked both (empty, 0.2 kg each)")
        belt_x = (s["conveyor"]["x_min"] + s["conveyor"]["x_max"]) / 2
        load = {"left": [belt_x, st["load"][1] + off, top],
                "right": [belt_x, st["load"][1] - off, top]}
        # carry pose: hands over where the belt slots will be, relative to the base
        carry = {s_: self.grasp_pose([load[s_][0], load[s_][1] - st["load"][1] + st["A"][1], top])
                 for s_ in SIDES}
        # Out of the row: a short straight pull clear of the row's other cans (4 mm
        # beside the pair), then the torso straight back up with the arms as they
        # are — the cans rise and come back with the chest. Folding the arms in
        # while still leaning is what swung the wrists: at this reach a straight
        # line back takes the wrist through its singularity (2-4 rad of swing),
        # and a joint move there turns the hanging can into its neighbour.
        now = {s_: self.tcp_now(s_) for s_ in SIDES}
        clear = s["container"]["W"] + 0.03
        self.dual_straight({s_: (now[s_][0] - clear, now[s_][1], now[s_][2]) for s_ in SIDES},
                           quats, 0.1)
        if preset != "home":
            self.torso_to("home")

        # --- conveyor, load station: both cans side by side on the belt
        quats = self.dual_move_to({s_: (carry[s_][0], carry[s_][1], carry[s_][2] + PRE_GRASP)
                                   for s_ in SIDES}, quats, "to the carry pose")
        self.drive_to(*st["load"], "conveyor load")
        self.place_pair(names, load, quats, "conveyor")

        # --- conveyor: carry to the filling station
        self.get_logger().info("conveyor running → filling station ...")
        self.conveyor_done.clear()
        self.conveyor_start.publish(Empty())
        if not self.conveyor_done.wait(timeout=60.0):
            raise StepFailed("the conveyor did not report delivery")
        unload = {s_: [p[0], p[1] + s["travel"], p[2]] for s_, p in load.items()}
        for s_ in SIDES:        # where the belt put them; bodies only, as pick targets
            self.objects[names[s_]] = ("can", unload[s_], True)
        self.refresh_scene()
        for s_ in SIDES:        # Gazebo recolours them itself; RViz draws the world file
            self.recolor_pub.publish(String(data=f"{names[s_]} {s['filled_rgba']}"))
        self.get_logger().info(f"delivered and filled: {s['filled_mass']} kg each (now red)")

        # --- unload station: pick the filled pair
        self.drive_to(*st["unload"], "conveyor unload")
        bottoms = {s_: self.locate(names[s_], unload[s_], tol=0.04) for s_ in SIDES}
        quats = self.pick_pair(names, bottoms, PRE_GRASP + 0.05, "filled")
        self.get_logger().info(f"picked both (filled, {s['filled_mass']} kg each)")
        # Curl the arms in first: the 8.4 kg comes toward the chest, which takes
        # moment off the hip and waist for the torso move and the drive.
        quats = self.tuck(quats)
        # Carry posture before driving: hips back, chest upright. (Moz1: at home with
        # the hands 0.53 m out the hip joint carries ~81 of its 100 N·m and the torso
        # folded under the drive; carry brings it to ~48 N·m — see robots/moz1.yaml.)
        self.torso_to("carry")

        # --- pallet B: into the open box on the deck (loaded: drive gently).
        # The torso goes down with the arms still curled, and only then do the arms
        # reach out over the box: the load is extended only once its centre of
        # gravity is low.
        self.drive_to(*st["B"], "pallet B", vmax=0.12, accel=0.15)
        slots = s["b_slots"]
        deck = {s_: (slots[s_][0], slots[s_][1], slots[s_][2] + 0.003) for s_ in SIDES}
        quats = None
        def reachable(torso):
            return all(self.dual_ik({s_: tuple(v + (dz if k == 2 else 0) for k, v in
                                               enumerate(self.grasp_pose(deck[s_])))
                                     for s_ in SIDES}, torso) is not None
                       for dz in (PRE_GRASP, 0.0))

        # Straight after the drive the reach check has failed in ~1 s for every
        # pose that passes moments later: let the base and the loaded torso settle
        # and re-check before giving up on a pose.
        self.settle("torso")
        for preset in self.deck_torsos:
            torso = self.presets[preset]
            for attempt in range(3):
                if reachable(torso):
                    break
                self.get_logger().info(f"  box not reachable with torso '{preset}' "
                                       f"(check {attempt + 1}/3)")
                time.sleep(1.0)
                self.refresh_scene()
            else:
                continue
            self.get_logger().info(f"pallet B box: torso '{preset}', arms curled")
            try:
                self.torso_to(preset)
                self.get_logger().info("  torso down: reaching out over the box")
                quats = self.above(slots, PRE_GRASP)
                break
            except StepFailed as exc:
                self.get_logger().info(f"  {exc} — trying the next torso pose")
        if quats is None:
            raise StepFailed("the box on pallet B is out of reach for every torso pose")
        self.place_pair(names, slots, quats, "pallet-B box")
        self.go_home()
        self.get_logger().info("done: both filled jerry cans verified in the box on pallet B")


def main():
    rclpy.init()
    node = TransferDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except StepFailed as exc:
        node.get_logger().error(f"stopped: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
