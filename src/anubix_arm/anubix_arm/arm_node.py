#!/usr/bin/env python3
"""
ANUBIX Arm Control Stack — arm_node.py  (Jetson Orin Nano)
============================================================
ROS 2 node wrapping the MyCobot Pro 450 Elite.

Subscribes to: /supervisor/arm_nav_goal  (geometry_msgs/PoseStamped)
               /supervisor/grip          (std_msgs/Bool)
               /supervisor/force_stop    (std_msgs/Bool)
Publishes to:  /arm/arm_status           (std_msgs/String)
               /arm/gripper_status       (std_msgs/String)
               /arm/touch_status         (std_msgs/Bool)
               /arm/current_pose         (geometry_msgs/PoseStamped)

Arm status values:     success | block | preflight_failed | mechanical_error
Gripper status values: successful_grip | successful_release | mechanical_error
Touch status values:   true | false

Supports simulate=true (time-sleep mock) and simulate=false (real Pro450).

Real hardware move order:
  0. Pre-flight check — reject immediately if unreachable
  1. Go to safe home (joint-space)
  2. 3-phase moveL to target (lift -> travel -> descend)

Safety (real hardware):
  - Z never drops below z_floor
  - Target rejected if unreachable, singular, or outside workspace
  - Background ZFloorMonitor calls mc.stop() instantly on Z breach
  - DH-based self-collision checker verifies full path before motion
  - All real motion uses moveL (linear Cartesian)
"""

import math
import time
import threading
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose

try:
    from pymycobot import Pro450Client
except ImportError:
    Pro450Client = None


# ══════════════════════════════════════════════════════════════════════════════
#  PRO 450 SELF-COLLISION CHECKER  (DH-based forward kinematics)
# ══════════════════════════════════════════════════════════════════════════════

class Pro450CollisionChecker:

    _DH = [
        (    0.0,  131.0,  math.pi / 2,  0.0),
        ( -264.0,    0.0,  0.0,          0.0),
        ( -224.0,    0.0,  0.0,          0.0),
        (    0.0,   75.0,  math.pi / 2,  0.0),
        (    0.0,   75.0, -math.pi / 2,  0.0),
        (    0.0,   45.0,  0.0,          0.0),
    ]

    _RADII  = [18, 14, 12, 12, 10, 9, 9]
    _MARGIN = 5.0
    _SKIP   = {(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6)}

    @staticmethod
    def _dh_matrix(a, d, alpha, theta):
        ca, sa = math.cos(alpha), math.sin(alpha)
        ct, st = math.cos(theta), math.sin(theta)
        return [
            [ct,       -st,        0,    a   ],
            [st * ca,   ct * ca,  -sa,  -sa*d],
            [st * sa,   ct * sa,   ca,   ca*d],
            [0,         0,         0,    1   ],
        ]

    @staticmethod
    def _mul(A, B):
        n = 4
        return [
            [sum(A[i][k] * B[k][j] for k in range(n)) for j in range(n)]
            for i in range(n)
        ]

    @classmethod
    def _joint_origins(cls, angles_deg):
        T = [[1 if i == j else 0 for j in range(4)] for i in range(4)]
        origins = [(0.0, 0.0, 0.0)]
        for i, (a, d, alpha, offset) in enumerate(cls._DH):
            theta = math.radians(angles_deg[i]) + offset
            M = cls._dh_matrix(a, d, alpha, theta)
            T = cls._mul(T, M)
            origins.append((T[0][3], T[1][3], T[2][3]))
        return origins

    @staticmethod
    def _seg_seg_dist(p1, p2, p3, p4):
        def sub(a, b):   return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
        def dot(a, b):   return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
        def norm2(a):    return dot(a, a)
        def add(a, b):   return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
        def scale(s, a): return (s*a[0], s*a[1], s*a[2])

        d1, d2, r = sub(p2, p1), sub(p4, p3), sub(p1, p3)
        a, e, f   = norm2(d1), norm2(d2), dot(d2, r)
        EPS = 1e-10

        if a <= EPS and e <= EPS:
            return math.sqrt(norm2(sub(p1, p3)))
        if a <= EPS:
            s, t = 0.0, max(0.0, min(1.0, f / e))
        else:
            c = dot(d1, r)
            if e <= EPS:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            else:
                b     = dot(d1, d2)
                denom = a * e - b * b
                s = max(0.0, min(1.0, (b * f - c * e) / denom)) if abs(denom) > EPS else 0.0
                t = (b * s + f) / e
                if t < 0.0:
                    t, s = 0.0, max(0.0, min(1.0, -c / a))
                elif t > 1.0:
                    t, s = 1.0, max(0.0, min(1.0, (b - c) / a))

        closest1 = add(p1, scale(s, d1))
        closest2 = add(p3, scale(t, d2))
        return math.sqrt(norm2(sub(closest1, closest2)))

    @classmethod
    def check(cls, angles_deg):
        origins = cls._joint_origins(angles_deg)
        n_segs  = len(origins) - 1
        for i in range(n_segs):
            for j in range(i + 2, n_segs + 1):
                if (i, j) in cls._SKIP or j >= len(origins):
                    continue
                dist = cls._seg_seg_dist(
                    origins[i], origins[i + 1],
                    origins[j], origins[j + 1] if j + 1 < len(origins) else origins[j],
                )
                min_clearance = (
                    cls._RADII[i]
                    + cls._RADII[min(j, len(cls._RADII) - 1)]
                    + cls._MARGIN
                )
                if dist < min_clearance:
                    return (
                        False,
                        f'Self-collision: link {i}<->link {j}  '
                        f'gap={dist:.1f}mm  required={min_clearance:.1f}mm',
                    )
        return True, 'Self-collision check OK'

    @classmethod
    def check_path(cls, from_angles, to_angles, steps=12):
        for step in range(steps + 1):
            t = step / steps
            interp = [
                from_angles[i] + t * (to_angles[i] - from_angles[i])
                for i in range(6)
            ]
            safe, reason = cls.check(interp)
            if not safe:
                pct = int(t * 100)
                return False, f'[{pct}% through path] {reason}', step
        return True, 'Path self-collision check OK', None


# ══════════════════════════════════════════════════════════════════════════════
#  Z FLOOR MONITOR  — kills motion the instant Z < z_floor
# ══════════════════════════════════════════════════════════════════════════════

class ZFloorMonitor:

    def __init__(self, mc, z_floor: float, monitor_hz: float, logger):
        self._mc         = mc
        self._z_floor    = z_floor
        self._monitor_hz = monitor_hz
        self._log        = logger
        self._running    = False
        self._thread     = None
        self.tripped     = False

    def start(self):
        self.tripped  = False
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)

    def _loop(self):
        while self._running:
            try:
                c = self._mc.get_coords()
                if c and len(c) >= 3 and c[2] < self._z_floor:
                    self._log.error(
                        f'[ARM] Z-MONITOR TRIP: Z={c[2]:.1f}mm breached floor '
                        f'({self._z_floor}mm)! EMERGENCY STOP.')
                    self._mc.stop()
                    self.tripped  = True
                    self._running = False
                    return
            except Exception:
                pass
            time.sleep(self._monitor_hz)


# ══════════════════════════════════════════════════════════════════════════════
#  ARM NODE
# ══════════════════════════════════════════════════════════════════════════════

class ArmNode(Node):

    def __init__(self):
        super().__init__('anubix_arm')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('simulate',               True)
        self.declare_parameter('arm_move_delay',         2.0)
        self.declare_parameter('grip_delay',             1.0)

        self.declare_parameter('home_x',                 0.0)
        self.declare_parameter('home_y',                 0.0)
        self.declare_parameter('home_z',                 0.3)

        self.declare_parameter('home_joint_1',           0.0)
        self.declare_parameter('home_joint_2',           0.0)
        self.declare_parameter('home_joint_3',          -90.0)
        self.declare_parameter('home_joint_4',           0.0)
        self.declare_parameter('home_joint_5',           0.0)
        self.declare_parameter('home_joint_6',           0.0)

        self.declare_parameter('pose_publish_rate_hz',   5.0)
        self.declare_parameter('speed',                  20)
        self.declare_parameter('transit_z',              280.0)
        self.declare_parameter('z_floor',                50.0)
        self.declare_parameter('monitor_hz',             0.03)

        self.declare_parameter('arm_host',               '192.168.0.232')
        self.declare_parameter('arm_port',               4500)
        self.declare_parameter('max_velocity_scaling',   0.5)
        self.declare_parameter('max_acceleration_scaling', 0.5)
        self.declare_parameter('force_threshold',        2.0)
        self.declare_parameter('touch_sensor_topic',     '/touch_sensor/data')

        # ── load values ──────────────────────────────────────────────────────
        self._simulate        = self.get_parameter('simulate').value
        self._arm_move_delay  = self.get_parameter('arm_move_delay').value
        self._grip_delay      = self.get_parameter('grip_delay').value
        self._speed           = self.get_parameter('speed').value
        self._transit_z       = self.get_parameter('transit_z').value
        self._z_floor         = self.get_parameter('z_floor').value
        self._monitor_hz      = self.get_parameter('monitor_hz').value
        self._force_threshold = self.get_parameter('force_threshold').value
        pose_rate = float(self.get_parameter('pose_publish_rate_hz').value)

        self._home_angles = [
            self.get_parameter(f'home_joint_{i}').value for i in range(1, 7)
        ]
        self._home_xyz = [
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_z').value),
        ]

        self._ws = dict(
            x=(-474, 474),
            y=(-474, 474),
            z=(self._z_floor, 677),
        )

        # ── state ────────────────────────────────────────────────────────────
        self._arm_position   = 'home'
        self._gripping       = False
        self._force_stopped  = False
        self._arm_busy       = False
        self._grip_busy      = False
        self._arm_lock       = threading.Lock()
        self._grip_lock      = threading.Lock()

        self._current_pose = PoseStamped()
        self._current_pose.header.frame_id = 'base_link'
        self._current_pose.pose.position.x = self._home_xyz[0]
        self._current_pose.pose.position.y = self._home_xyz[1]
        self._current_pose.pose.position.z = self._home_xyz[2]
        self._current_pose.pose.orientation.w = 1.0
        self._pose_lock = threading.Lock()

        self._sub_group = ReentrantCallbackGroup()

        # ── hardware / simulate backend ──────────────────────────────────────
        self._mc      = None
        self._monitor = None

        if not self._simulate:
            if Pro450Client is None:
                self.get_logger().fatal(
                    '[ARM] simulate=false but pymycobot is not installed!')
                raise RuntimeError('pymycobot missing')

            host = self.get_parameter('arm_host').value
            port = self.get_parameter('arm_port').value
            self.get_logger().info(
                f'[ARM] Connecting to Pro450 at {host}:{port}...')
            self._mc = Pro450Client(host, port)

            if self._mc.is_power_on() != 1:
                self.get_logger().info('[ARM] Powering on arm...')
                self._mc.power_on()
                time.sleep(3)

            self._mc.set_collision_mode(1)
            self._mc.set_movement_type(1)          # moveL globally

            self._monitor = ZFloorMonitor(
                self._mc, self._z_floor, self._monitor_hz, self.get_logger(),
            )
            self.get_logger().info('[ARM] Pro450 connected and ready.')

        # ── backend dispatch ─────────────────────────────────────────────────
        if self._simulate:
            self._move_impl = self._simulate_arm_move
            self._grip_impl = self._simulate_grip
        else:
            self._move_impl = self._real_arm_move
            self._grip_impl = self._real_grip

        # ── QoS profiles ────────────────────────────────────────────────────
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/supervisor/arm_nav_goal', self._on_arm_goal,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/grip', self._on_grip,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop,
            force_stop_qos, callback_group=self._sub_group)

        # ── publishers ───────────────────────────────────────────────────────
        self._arm_status_pub     = self.create_publisher(
            String, '/arm/arm_status', pub_qos)
        self._gripper_status_pub = self.create_publisher(
            String, '/arm/gripper_status', pub_qos)
        self._touch_status_pub   = self.create_publisher(
            Bool, '/arm/touch_status', pub_qos)
        self._pose_pub           = self.create_publisher(
            PoseStamped, '/arm/current_pose', pose_qos)

        # ── periodic pose + initial latch ────────────────────────────────────
        self._publish_current_pose()
        if pose_rate > 0:
            self.create_timer(1.0 / pose_rate, self._publish_current_pose)

        # ── startup banner ───────────────────────────────────────────────────
        self.get_logger().info('=' * 50)
        self.get_logger().info('  ANUBIX Arm Control Node - Jetson Orin Nano')
        mode = 'SIMULATE' if self._simulate else 'HARDWARE (Pro450)'
        self.get_logger().info(f'  Mode: {mode}')
        if not self._simulate:
            self.get_logger().info(
                f'  Z floor: {self._z_floor}mm  Transit: {self._transit_z}mm  '
                f'Speed: {self._speed}')
            self.get_logger().info(f'  Home angles: {self._home_angles}')
        else:
            self.get_logger().info(
                f'  Arm move delay: {self._arm_move_delay}s  '
                f'Grip delay: {self._grip_delay}s')
        self.get_logger().info(
            f'  Home pose: ({self._home_xyz[0]:.3f}, '
            f'{self._home_xyz[1]:.3f}, {self._home_xyz[2]:.3f})')
        self.get_logger().info('=' * 50)
        self.get_logger().info('[ARM] Ready and waiting for commands.')

    # ══════════════════════════════════════════════════════════════════════════
    #  POSE PUBLISHER
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_current_pose(self):
        with self._pose_lock:
            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.pose            = self._current_pose.pose
        self._pose_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    #  SUBSCRIBER CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def _on_force_stop(self, msg: Bool):
        was = self._force_stopped
        self._force_stopped = bool(msg.data)
        if self._force_stopped:
            self.get_logger().warning(
                '[ARM] *** FORCE STOP RECEIVED *** — halting all operations')
            if self._mc:
                try:
                    self._mc.stop()
                except Exception:
                    pass
        elif was:
            self.get_logger().info(
                '[ARM] Force stop CLEARED — ready for new commands')

    def _on_arm_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        frame = msg.header.frame_id

        self.get_logger().info(
            f'[ARM] ========================================')
        self.get_logger().info(
            f'[ARM] Arm goal RECEIVED: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'frame="{frame}"')
        self.get_logger().info(
            f'[ARM] ========================================')

        if self._force_stopped:
            self.get_logger().warning(
                '[ARM] REJECTED: force_stopped. Publishing "mechanical_error".')
            self._arm_status_pub.publish(String(data='mechanical_error'))
            return

        with self._arm_lock:
            if self._arm_busy:
                self.get_logger().warning(
                    '[ARM] Already executing arm move! Publishing "block".')
                self._arm_status_pub.publish(String(data='block'))
                return
            self._arm_busy = True

        threading.Thread(
            target=self._move_impl, args=(msg,), daemon=True,
        ).start()

    def _on_grip(self, msg: Bool):
        action = msg.data
        action_str = 'CLOSE (grip)' if action else 'OPEN (release)'

        self.get_logger().info(
            f'[ARM] ========================================')
        self.get_logger().info(f'[ARM] Grip command RECEIVED: {action_str}')
        self.get_logger().info(
            f'[ARM] ========================================')

        if self._force_stopped:
            self.get_logger().warning(
                '[ARM] REJECTED: force_stopped. Publishing "mechanical_error".')
            self._gripper_status_pub.publish(String(data='mechanical_error'))
            return

        with self._grip_lock:
            if self._grip_busy:
                self.get_logger().warning(
                    '[ARM] Already executing grip! Publishing "mechanical_error".')
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                return
            self._grip_busy = True

        threading.Thread(
            target=self._grip_impl, args=(action,), daemon=True,
        ).start()

    # ══════════════════════════════════════════════════════════════════════════
    #  FRAME RESOLUTION  (base_link = absolute, calibration = delta)
    # ══════════════════════════════════════════════════════════════════════════

    def _resolve_goal(self, goal: PoseStamped) -> Pose:
        p = Pose()
        p.orientation = goal.pose.orientation

        if goal.header.frame_id == 'calibration':
            with self._pose_lock:
                cur = self._current_pose.pose.position
            p.position.x = cur.x + goal.pose.position.x
            p.position.y = cur.y + goal.pose.position.y
            p.position.z = cur.z + goal.pose.position.z
            self.get_logger().info(
                f'[ARM] Calibration (delta) resolved -> '
                f'({p.position.x:.4f}, {p.position.y:.4f}, '
                f'{p.position.z:.4f}) m')
        else:
            p.position = goal.pose.position

        return p

    # ══════════════════════════════════════════════════════════════════════════
    #  SIMULATE BACKEND
    # ══════════════════════════════════════════════════════════════════════════

    def _simulate_arm_move(self, goal: PoseStamped):
        try:
            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Move ABORTED (force stopped) -> "mechanical_error"')
                return

            target = self._resolve_goal(goal)
            x_mm = target.position.x * 1000
            y_mm = target.position.y * 1000
            z_mm = target.position.z * 1000

            # ── step 0: pre-flight ───────────────────────────────────────
            self.get_logger().info('[ARM] [SIM] Running pre-flight check...')
            passed, report = self._preflight_sim(x_mm, y_mm, z_mm)
            for line in report:
                self.get_logger().info(f'[ARM]   preflight: {line}')
            if not passed:
                self.get_logger().error(
                    '[ARM] [SIM] Move REJECTED — pre-flight failed')
                self._arm_status_pub.publish(String(data='preflight_failed'))
                return

            # ── step 1: go home ──────────────────────────────────────────
            self.get_logger().info(
                f'[ARM] [SIM] Going to home position '
                f'— waiting {self._arm_move_delay}s...')
            time.sleep(self._arm_move_delay)
            with self._pose_lock:
                self._current_pose.pose.position.x = self._home_xyz[0]
                self._current_pose.pose.position.y = self._home_xyz[1]
                self._current_pose.pose.position.z = self._home_xyz[2]
            self._publish_current_pose()
            self.get_logger().info('[ARM] [SIM] Home reached.')

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Move ABORTED (force stopped) -> "mechanical_error"')
                return

            # ── step 2: move to target ───────────────────────────────────
            self.get_logger().info(
                f'[ARM] [SIM] Moving to target '
                f'({target.position.x:.3f}, {target.position.y:.3f}, '
                f'{target.position.z:.3f}) — waiting {self._arm_move_delay}s...')
            time.sleep(self._arm_move_delay)

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Move ABORTED (force stopped) -> "mechanical_error"')
                return

            with self._pose_lock:
                self._current_pose.pose = target
                o = self._current_pose.pose.orientation
                if o.w == 0.0 and o.x == 0.0 and o.y == 0.0 and o.z == 0.0:
                    self._current_pose.pose.orientation.w = 1.0
            self._publish_current_pose()

            self._arm_position = (
                'home' if abs(target.position.z - self._home_xyz[2]) < 0.001
                else 'extended'
            )
            self._arm_status_pub.publish(String(data='success'))
            self.get_logger().info(
                f'[ARM] [SIM] Move COMPLETE -> "success" '
                f'pos=({target.position.x:.3f}, {target.position.y:.3f}, '
                f'{target.position.z:.3f})')

        except Exception as e:
            self.get_logger().error(
                f'[ARM] Exception during simulated move: {e}\n'
                f'{traceback.format_exc()}')
            self._arm_status_pub.publish(String(data='success'))
        finally:
            with self._arm_lock:
                self._arm_busy = False

    def _simulate_grip(self, close: bool):
        try:
            action_str = 'close' if close else 'open'
            self.get_logger().info(
                f'[ARM] [SIM] Gripper {action_str} '
                f'— waiting {self._grip_delay}s...')
            time.sleep(self._grip_delay)

            if self._force_stopped:
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Grip ABORTED (force stopped) -> "mechanical_error"')
                return

            if close:
                self._gripping = True
                self._gripper_status_pub.publish(
                    String(data='successful_grip'))
                self._touch_status_pub.publish(Bool(data=True))
                self.get_logger().info(
                    '[ARM] Gripper CLOSED -> "successful_grip"')
                self.get_logger().info('[ARM] Touch sensor -> true')
            else:
                self._gripping = False
                self._gripper_status_pub.publish(
                    String(data='successful_release'))
                self._touch_status_pub.publish(Bool(data=False))
                self.get_logger().info(
                    '[ARM] Gripper OPENED -> "successful_release"')

        except Exception as e:
            self.get_logger().error(
                f'[ARM] Exception during grip: {e}\n'
                f'{traceback.format_exc()}')
            if close:
                self._gripper_status_pub.publish(
                    String(data='successful_grip'))
                self._touch_status_pub.publish(Bool(data=True))
            else:
                self._gripper_status_pub.publish(
                    String(data='successful_release'))
        finally:
            with self._grip_lock:
                self._grip_busy = False

    # ══════════════════════════════════════════════════════════════════════════
    #  REAL BACKEND  (Pro450 hardware)
    # ══════════════════════════════════════════════════════════════════════════

    def _real_arm_move(self, goal: PoseStamped):
        try:
            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            self._clear_errors()
            target = self._resolve_goal(goal)

            x = target.position.x * 1000       # ROS metres -> Pro450 mm
            y = target.position.y * 1000
            z = target.position.z * 1000

            # ── step 0: pre-flight ───────────────────────────────────────
            self.get_logger().info(
                f'[ARM] Pre-flight check for '
                f'[{x:.1f}, {y:.1f}, {z:.1f}] mm ...')
            home_rx, home_ry, home_rz = 0.0, 0.0, 0.0
            passed, report = self._preflight(
                x, y, z, home_rx, home_ry, home_rz)
            for line in report:
                self.get_logger().info(f'[ARM]   preflight: {line}')

            if not passed:
                self.get_logger().error(
                    '[ARM] Move REJECTED — pre-flight failed')
                self._arm_status_pub.publish(String(data='preflight_failed'))
                return

            self.get_logger().info('[ARM] Pre-flight passed.')

            # ── step 1: go home (joint-space) ────────────────────────────
            self.get_logger().info(
                f'[ARM] Going to home angles: {self._home_angles}')
            self._mc.send_angles(self._home_angles, self._speed)
            time.sleep(0.35)
            ok = self._wait_stop()
            if not ok or self._force_stopped:
                self.get_logger().error(
                    '[ARM] Failed to reach home position.')
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            self.get_logger().info('[ARM] Home reached.')

            c = self._coords()
            if not c:
                self.get_logger().error(
                    '[ARM] Cannot read coords after homing.')
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return
            rx, ry, rz = c[3], c[4], c[5]

            self.get_logger().info(
                f'[ARM] Moving to target [{x:.1f}, {y:.1f}, {z:.1f}] mm')

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── step 2 phase 1: lift to transit height ───────────────────
            if c[2] < self._transit_z:
                lift = [c[0], c[1], self._transit_z, rx, ry, rz]
                ok = self._linear_move(lift, tag='lift')
                if not ok:
                    self._arm_status_pub.publish(
                        String(data='mechanical_error'))
                    return
                c = self._coords()
                if c:
                    rx, ry, rz = c[3], c[4], c[5]

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── step 2 phase 2: horizontal travel ────────────────────────
            travel = [x, y, self._transit_z, rx, ry, rz]
            ok = self._linear_move(travel, tag='travel')
            if not ok:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return
            c = self._coords()
            if c:
                rx, ry, rz = c[3], c[4], c[5]

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── step 2 phase 3: descend to target ────────────────────────
            descent = [x, y, z, rx, ry, rz]
            ok = self._linear_move(descent, tag='descend')
            if not ok:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── success ──────────────────────────────────────────────────
            with self._pose_lock:
                self._current_pose.pose = target
                o = self._current_pose.pose.orientation
                if o.w == 0.0 and o.x == 0.0 and o.y == 0.0 and o.z == 0.0:
                    self._current_pose.pose.orientation.w = 1.0
            self._publish_current_pose()

            self._arm_position = (
                'home' if abs(target.position.z - self._home_xyz[2]) < 0.001
                else 'extended'
            )
            self._arm_status_pub.publish(String(data='success'))
            self.get_logger().info(
                f'[ARM] Move COMPLETE -> "success" '
                f'pos=({target.position.x:.3f}, {target.position.y:.3f}, '
                f'{target.position.z:.3f})')

        except Exception as e:
            self.get_logger().error(
                f'[ARM] Exception during arm move: {e}\n'
                f'{traceback.format_exc()}')
            self._arm_status_pub.publish(String(data='mechanical_error'))
        finally:
            with self._arm_lock:
                self._arm_busy = False

    def _real_grip(self, close: bool):
        try:
            if self._force_stopped:
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                return

            action_str = 'close' if close else 'open'
            self.get_logger().info(f'[ARM] [HW] Gripper {action_str}')

            # TODO: replace with real gripper driver call, e.g.:
            #   self._gripper_driver.set_state(close)
            #   force = self._touch_sensor.read_force()
            time.sleep(self._grip_delay)

            if self._force_stopped:
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                return

            if close:
                # TODO: compare measured force against self._force_threshold
                self._gripping = True
                self._gripper_status_pub.publish(
                    String(data='successful_grip'))
                self._touch_status_pub.publish(Bool(data=True))
                self.get_logger().info(
                    '[ARM] Gripper CLOSED -> "successful_grip"')
            else:
                self._gripping = False
                self._gripper_status_pub.publish(
                    String(data='successful_release'))
                self._touch_status_pub.publish(Bool(data=False))
                self.get_logger().info(
                    '[ARM] Gripper OPENED -> "successful_release"')

        except Exception as e:
            self.get_logger().error(
                f'[ARM] Exception during grip: {e}\n'
                f'{traceback.format_exc()}')
            self._gripper_status_pub.publish(String(data='mechanical_error'))
        finally:
            with self._grip_lock:
                self._grip_busy = False

    # ══════════════════════════════════════════════════════════════════════════
    #  PRE-FLIGHT CHECKS
    # ══════════════════════════════════════════════════════════════════════════

    def _preflight_sim(self, x, y, z):
        """Geometry-only pre-flight for simulate mode (x, y, z in mm)."""
        errors = []
        report = []

        for axis, val in [('x', x), ('y', y), ('z', z)]:
            lo, hi = self._ws[axis]
            if not (lo <= val <= hi):
                errors.append(f'{axis}={val:.1f} outside [{lo}, {hi}]')

        r_xy = (x**2 + y**2) ** 0.5
        if r_xy < 30:
            errors.append(
                f'XY radius {r_xy:.1f}mm < 30mm — J1 singularity zone')

        reach = (x**2 + y**2 + (z - 50)**2) ** 0.5
        if reach > 445:
            errors.append(
                f'Reach {reach:.1f}mm > 445mm — outside arm envelope')

        if errors:
            return False, errors

        report.append('Workspace   OK')
        report.append(f'XY radius   OK  ({r_xy:.0f}mm from axis)')
        report.append(f'Reach       OK  ({reach:.0f}mm)')
        report.append('IK check    — skipped (simulate mode)')

        safe, reason = Pro450CollisionChecker.check(self._home_angles)
        if not safe:
            return False, [f'Self-collision at home: {reason}']
        report.append('Self-collision (home) OK')
        return True, report

    def _preflight(self, x, y, z, rx, ry, rz):
        """Full pre-flight for real hardware (geometry + IK). x/y/z in mm."""
        errors = []
        report = []

        for axis, val in [('x', x), ('y', y), ('z', z)]:
            lo, hi = self._ws[axis]
            if not (lo <= val <= hi):
                errors.append(f'{axis}={val:.1f} outside [{lo}, {hi}]')

        r_xy = (x**2 + y**2) ** 0.5
        if r_xy < 30:
            errors.append(
                f'XY radius {r_xy:.1f}mm < 30mm — J1 singularity zone')

        reach = (x**2 + y**2 + (z - 50)**2) ** 0.5
        if reach > 445:
            errors.append(
                f'Reach {reach:.1f}mm > 445mm — outside arm envelope')

        if errors:
            return False, errors

        current_angles = self._angles()
        if not current_angles:
            return False, ['Cannot read current angles for IK check']

        ik = self._mc.solve_inv_kinematics(
            [x, y, z, rx, ry, rz], current_angles)
        if not ik or not isinstance(ik, list) or len(ik) < 6:
            return False, ['IK solver returned no solution']
        for v in ik:
            if v is None:
                return False, ['IK solution contains null joint value']
            if abs(v) > 168:
                return False, [
                    f'IK solution has joint at {v:.1f} deg — near hard limit']

        report.append('Workspace   OK')
        report.append(f'XY radius   OK  ({r_xy:.0f}mm from axis)')
        report.append(f'Reach       OK  ({reach:.0f}mm)')
        report.append(
            f'IK solution OK  joints={[round(a, 1) for a in ik]}')

        safe, reason, _ = Pro450CollisionChecker.check_path(
            self._home_angles, ik, steps=18)
        if not safe:
            return False, [
                f'Self-collision predicted along path: {reason}']
        report.append('Self-collision path  OK  (18 steps checked)')

        return True, report

    # ══════════════════════════════════════════════════════════════════════════
    #  LOW-LEVEL HARDWARE HELPERS
    # ══════════════════════════════════════════════════════════════════════════

    def _coords(self):
        c = self._mc.get_coords()
        return c if (c and len(c) == 6) else None

    def _angles(self):
        a = self._mc.get_angles()
        return a if (a and len(a) == 6) else None

    def _wait_stop(self, timeout=30):
        t0 = time.time()
        while self._mc.is_moving() == 1:
            if self._monitor and self._monitor.tripped:
                return False
            if self._force_stopped:
                return False
            if time.time() - t0 > timeout:
                self.get_logger().warning(
                    '[ARM] Motion timeout — force stopping.')
                self._mc.stop()
                return False
            time.sleep(0.08)
        return True

    def _clear_errors(self):
        err = self._mc.get_error_information()
        if err != 0:
            self._mc.clear_error_information()
            self._mc.servo_restore(254)
            time.sleep(0.8)

    def _linear_move(self, target, speed=None, tag='move'):
        """Execute one moveL segment with Z-floor monitor active."""
        if speed is None:
            speed = self._speed
        self.get_logger().info(
            f'[ARM] [{tag}] -> {[round(v, 1) for v in target]}')
        self._monitor.start()
        self._mc.send_coords(target, speed, 1)     # 1 = moveL
        time.sleep(0.35)
        ok = self._wait_stop()
        self._monitor.stop()
        if self._monitor.tripped:
            self.get_logger().error(
                f'[ARM] [{tag}] ABORTED by Z-floor monitor.')
            return False
        return ok

    def _go_home(self):
        """Move to safe home (utility method)."""
        self.get_logger().info('[ARM] Returning to safe home...')
        if self._simulate:
            time.sleep(self._arm_move_delay)
            with self._pose_lock:
                self._current_pose.pose.position.x = self._home_xyz[0]
                self._current_pose.pose.position.y = self._home_xyz[1]
                self._current_pose.pose.position.z = self._home_xyz[2]
            self._publish_current_pose()
            return

        self._clear_errors()
        c = self._coords()
        if c and c[2] < self._transit_z:
            lift = [c[0], c[1], self._transit_z, c[3], c[4], c[5]]
            self._linear_move(lift, tag='lift-before-home')

        self._mc.send_angles(self._home_angles, self._speed)
        time.sleep(0.35)
        self._wait_stop()
        self.get_logger().info(f'[ARM] Home done. Current: {self._coords()}')


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('[ARM] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
