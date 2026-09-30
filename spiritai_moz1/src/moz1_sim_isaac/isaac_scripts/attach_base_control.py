"""
attach_base_control.py  --  Moz1 Isaac Sim base velocity controller (Phase 2).

Paste into Isaac Sim 5.1.0 >> Window > Script Editor and Run, with the sim
PLAYING. Subscribes to the SAME topic Nav2 publishes on the real robot --

  SUB /cmd_vel   geometry_msgs/Twist   (Nav2 MPPI Omni output: vx, vy, wz)

-- and converts the holonomic twist to four X-drive wheel target velocities via
mecanum IK, then writes them to the wheel joint drives each physics step. This
replaces the real cmd_vel_bridge + the embedded controller's base loop for sim.
(That loop lives on the controller at 172.16.0.20 -- MovaX is a TCP operator
console, not a ROS node.)

Ported from spirit_mecanum_pos_paste.py; the ONLY contract change is the topic
(/spirit/cmd_vel -> /cmd_vel) so the unchanged Nav2 stack drives it. Safety:
0.3 s command timeout, slew-rate limiting, deadzone.

Re-runnable: cleans up the previous controller via builtins handles.
"""

import builtins
import math
import threading
import time

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except ImportError:
    raise RuntimeError("rclpy not found -- source ROS2 Humble before launching Isaac Sim.")

# ------------------------------------------------------------------ #
# CONFIG  (keep in sync with interface_config.py; verify with discover_prims.py)
# ------------------------------------------------------------------ #
JOINTS_PATH = "/World/Moz1_test_environment/Moz1_omni_gripper_full/MOZ1/joints"
CMD_VEL_TOPIC = "/cmd_vel"

WHEEL_RADIUS = 0.08
WHEEL_RADIAL_DIST = math.sqrt(2) * 0.2423

CMD_TIMEOUT = 0.3        # s; matches the real cmd_vel_bridge staleness guard intent
MAX_WHEEL_SPEED = 30.0   # rad/s
MAX_WHEEL_ACCEL = 60.0   # rad/s^2
CMD_DEADZONE = 1e-3
RAD_TO_DEG = 180.0 / math.pi

# X-drive wheel layout (re-verify signs/angles against the sim USD):
#   Base_0 Back-Left (-x,+y) 135 deg, fwd negative
#   Base_1 Back-Right(-x,-y) -135 deg, fwd positive
#   Base_2 Front-Right(+x,-y) -45 deg, fwd positive
#   Base_3 Front-Left (+x,+y)  45 deg, fwd negative
WHEELS = [
    {"joint": "Base_0", "theta": math.radians(135.0), "sign": +1.0},
    {"joint": "Base_1", "theta": math.radians(-135.0), "sign": -1.0},
    {"joint": "Base_2", "theta": math.radians(-45.0), "sign": +1.0},
    {"joint": "Base_3", "theta": math.radians(45.0), "sign": -1.0},
]

# ------------------------------------------------------------------ #
# Clean up a previous run
# ------------------------------------------------------------------ #
for _name, _action in [
    ("_moz1_base_physics_sub", lambda x: x.unsubscribe()),
    ("_moz1_base_timeline_sub", lambda x: None),
    ("_moz1_base_stop_event", lambda x: x.set()),
    ("_moz1_base_executor", lambda x: x.shutdown(wait_for_running_callbacks=False)),
    ("_moz1_base_ros_node", lambda x: x.destroy_node()),
]:
    if getattr(builtins, _name, None):
        try:
            _action(getattr(builtins, _name))
        except Exception:
            pass
        setattr(builtins, _name, None)
if getattr(builtins, "_moz1_base_ros_thread", None):
    builtins._moz1_base_ros_thread.join(timeout=1.5)
    builtins._moz1_base_ros_thread = None
print("[base] Previous controller cleaned up")

# ------------------------------------------------------------------ #
# State + IK
# ------------------------------------------------------------------ #
_lock = threading.Lock()
_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "t": time.time()}
_wheel_target = [0.0, 0.0, 0.0, 0.0]


def _wheel_speeds_from_twist(vx, vy, wz):
    L, R = WHEEL_RADIAL_DIST, WHEEL_RADIUS
    out = []
    for w in WHEELS:
        v_along_roll = vx * math.cos(w["theta"]) + vy * math.sin(w["theta"]) + wz * L
        speed = (v_along_roll / R) * w["sign"]
        out.append(max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, speed)))
    return out


def _slew(cur, tgt, max_delta):
    diff = tgt - cur
    return tgt if abs(diff) <= max_delta else cur + math.copysign(max_delta, diff)


def _on_physics_step(dt):
    global _wheel_target
    with _lock:
        vx, vy, wz, last = _cmd["vx"], _cmd["vy"], _cmd["wz"], _cmd["t"]
    if (time.time() - last) > CMD_TIMEOUT:
        vx = vy = wz = 0.0
    if abs(vx) < CMD_DEADZONE and abs(vy) < CMD_DEADZONE and abs(wz) < CMD_DEADZONE:
        target = [0.0] * 4
    else:
        target = _wheel_speeds_from_twist(vx, vy, wz)
    max_delta = MAX_WHEEL_ACCEL * dt
    new_target = [_slew(_wheel_target[i], target[i], max_delta) for i in range(4)]

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return
    for i, w in enumerate(WHEELS):
        prim = stage.GetPrimAtPath(f"{JOINTS_PATH}/{w['joint']}")
        if not prim.IsValid():
            continue
        attr = prim.GetAttribute("drive:angular:physics:targetVelocity")
        if attr:
            attr.Set(float(new_target[i] * RAD_TO_DEG))
    _wheel_target = new_target


class _CmdVelNode(Node):
    def __init__(self):
        super().__init__("moz1_sim_base_controller")
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._cb, 10)
        self.get_logger().info(f"Listening on {CMD_VEL_TOPIC}")

    def _cb(self, msg):
        with _lock:
            _cmd.update(vx=float(msg.linear.x), vy=float(msg.linear.y),
                        wz=float(msg.angular.z), t=time.time())


def _spin(executor, stop_event):
    while not stop_event.is_set():
        executor.spin_once(timeout_sec=0.05)


def _on_timeline(e):
    global _wheel_target
    t = omni.timeline.TimelineEventType
    if e.type in (t.STOP, t.PLAY):
        with _lock:
            _cmd.update(vx=0.0, vy=0.0, wz=0.0, t=time.time())
        _wheel_target = [0.0, 0.0, 0.0, 0.0]
        print("[base] Timeline reset")


# ------------------------------------------------------------------ #
# Startup
# ------------------------------------------------------------------ #
if not rclpy.ok():
    rclpy.init()
_node = _CmdVelNode()
_executor = SingleThreadedExecutor()
_executor.add_node(_node)
_stop = threading.Event()
_thread = threading.Thread(target=_spin, args=(_executor, _stop), daemon=True,
                           name="moz1_base_spin")
_thread.start()
_physics_sub = get_physx_interface().subscribe_physics_step_events(_on_physics_step)
try:
    _tl = omni.timeline.get_timeline_interface()
    builtins._moz1_base_timeline_sub = _tl.get_timeline_event_stream(
    ).create_subscription_to_pop(_on_timeline, name="moz1_base_timeline")
except Exception as ex:
    print(f"[base] Timeline sub skipped: {ex}")

builtins._moz1_base_physics_sub = _physics_sub
builtins._moz1_base_stop_event = _stop
builtins._moz1_base_ros_thread = _thread
builtins._moz1_base_executor = _executor
builtins._moz1_base_ros_node = _node
print(f"[base] Controller running -- SUB {CMD_VEL_TOPIC} (holonomic vx,vy,wz)")
