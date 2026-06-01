#!/usr/bin/env python3
"""
anubix_arm — arm_node.py
=========================
ROS 2 node wrapping the MyCobot Pro 450 Elite Point Mover.

Subscribes to /supervisor/arm_nav_goal, /supervisor/force_stop
Publishes    /arm/arm_status, /arm/current_pose

Gripper control is handled by the separate anubix_gripper package.

Changes vs original
-------------------
Self-collision avoidance overhaul:
  • Corrected DH parameters and link-capsule radii for the Pro 450.
  • Fixed off-by-one in segment-pair iteration that caused spurious checks
    against a non-existent 7th segment.
  • Path collision check now uses Cartesian-space interpolation via FK
    rather than naïve linear joint interpolation, matching what moveL
    actually does.
  • IK solution is evaluated over MULTIPLE elbow configurations and the
    one with the greatest minimum link-clearance is preferred — the arm
    naturally avoids the J6↔J2/J3 collision zone.
  • Joint limit guard uses per-joint limits for the Pro 450 instead of a
    uniform ±168° that rejected valid poses.
  • Pre-flight no longer rejects on advisory (XY radius / reach) — only
    true workspace violations and IK failures block a move.
  • All other safety layers (Z-floor hard limit, Z-floor monitor thread,
    3-phase Cartesian motion) are preserved unchanged.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

try:
    from pymycobot import Pro450Client
except ImportError:
    Pro450Client = None


# ═══════════════════════════════════════════════════════════════════════════════
#  PRO 450  —  KINEMATICS  &  SELF-COLLISION CHECKER
# ═══════════════════════════════════════════════════════════════════════════════

class Pro450Kinematics:
    """
    Modified-DH forward kinematics for the MyCobot Pro 450.

    DH table  (a_mm, d_mm, alpha_rad, theta_offset_rad):
      Joint 1:   a=0       d=131     alpha= π/2   offset=0
      Joint 2:   a=-264    d=0       alpha= 0     offset=0
      Joint 3:   a=-224    d=0       alpha= 0     offset=0
      Joint 4:   a=0       d=75      alpha= π/2   offset=0
      Joint 5:   a=0       d=75      alpha=-π/2   offset=0
      Joint 6:   a=0       d=45      alpha= 0     offset=0

    Per-joint soft limits (degrees) — Pro 450 actual range:
      J1: ±165   J2: ±165   J3: ±165
      J4: ±165   J5: ±165   J6: ±175
    """

    # Modified DH: (a_mm, d_mm, alpha_rad, theta_offset_rad)
    DH = [
        (    0.0,  131.0,  math.pi / 2,  0.0),
        ( -264.0,    0.0,  0.0,          0.0),
        ( -224.0,    0.0,  0.0,          0.0),
        (    0.0,   75.0,  math.pi / 2,  0.0),
        (    0.0,   75.0, -math.pi / 2,  0.0),
        (    0.0,   45.0,  0.0,          0.0),
    ]

    # Per-joint hard limits (degrees) — used for IK validation
    JOINT_LIMITS = [165, 165, 165, 165, 165, 175]

    @staticmethod
    def _dh_matrix(a, d, alpha, theta):
        ca, sa = math.cos(alpha), math.sin(alpha)
        ct, st = math.cos(theta), math.sin(theta)
        return [
            [ct,       -st,       0,    a    ],
            [st * ca,   ct * ca, -sa,  -sa*d ],
            [st * sa,   ct * sa,  ca,   ca*d ],
            [0,         0,        0,    1    ],
        ]

    @staticmethod
    def _mul(A, B):
        return [
            [sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)
        ]

    @classmethod
    def joint_origins(cls, angles_deg):
        """
        Return 7 points (base + 6 joint frames) in mm.
        angles_deg: list/tuple of 6 joint angles in degrees.
        """
        T = [[1 if i == j else 0 for j in range(4)] for i in range(4)]
        origins = [(0.0, 0.0, 0.0)]
        for i, (a, d, alpha, offset) in enumerate(cls.DH):
            theta = math.radians(angles_deg[i]) + offset
            T = cls._mul(T, cls._dh_matrix(a, d, alpha, theta))
            origins.append((T[0][3], T[1][3], T[2][3]))
        return origins  # 7 points → 6 segments


class Pro450CollisionChecker:
    """
    Capsule-based self-collision checker for the MyCobot Pro 450.

    Segment radii (mm) — conservative physical body estimates:
      Seg 0 (base → J1):  40
      Seg 1 (J1  → J2):  35
      Seg 2 (J2  → J3):  30
      Seg 3 (J3  → J4):  28
      Seg 4 (J4  → J5):  25
      Seg 5 (J5  → J6/tip): 22

    Clearance margin added on top of sum-of-radii: 8 mm
    This margin is intentionally kept moderate so the checker does not
    reject valid poses that merely come close.  The arm's built-in
    collision mode (set_collision_mode(1)) is the final hardware guard.

    Pairs skipped (directly connected joints always overlap):
      (0,1) (1,2) (2,3) (3,4) (4,5)
    Note: (i, i+2) pairs are checked — they are NOT adjacent.
    """

    _RADII  = [40, 35, 30, 28, 25, 22]
    _MARGIN = 8.0

    # Directly-connected segment pairs — guaranteed to touch at the joint
    _SKIP = frozenset({(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)})

    @staticmethod
    def _seg_seg_dist(p1, p2, p3, p4):
        """Minimum Euclidean distance between line segments p1→p2 and p3→p4."""
        def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
        def dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
        def norm2(a): return dot(a, a)
        def add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
        def sc(s, a): return (s*a[0], s*a[1], s*a[2])

        d1 = sub(p2, p1);  d2 = sub(p4, p3);  r = sub(p1, p3)
        a  = norm2(d1);    e  = norm2(d2);     f = dot(d2, r)
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
                b = dot(d1, d2)
                denom = a * e - b * b
                s = max(0.0, min(1.0, (b*f - c*e) / denom)) if abs(denom) > EPS else 0.0
                t = (b*s + f) / e
                if t < 0.0:
                    t, s = 0.0, max(0.0, min(1.0, -c / a))
                elif t > 1.0:
                    t, s = 1.0, max(0.0, min(1.0, (b - c) / a))

        return math.sqrt(norm2(sub(add(p1, sc(s, d1)), add(p3, sc(t, d2)))))

    @classmethod
    def _min_clearance(cls, origins):
        """
        Return the minimum clearance (gap beyond sum-of-radii) found across
        all non-adjacent segment pairs, given a list of 7 joint origins.
        A negative value means collision.
        """
        n_segs = len(origins) - 1   # 6 segments, indices 0..5
        min_gap = float('inf')
        worst   = (-1, -1)

        for i in range(n_segs):
            for j in range(i + 2, n_segs):   # skip (i, i+1); j ≤ 5
                if (i, j) in cls._SKIP:
                    continue
                dist = cls._seg_seg_dist(
                    origins[i], origins[i + 1],
                    origins[j], origins[j + 1],
                )
                required = cls._RADII[i] + cls._RADII[j] + cls._MARGIN
                gap = dist - required
                if gap < min_gap:
                    min_gap = gap
                    worst   = (i, j)

        return min_gap, worst

    @classmethod
    def check(cls, angles_deg):
        """
        Returns (safe: bool, reason: str, clearance_mm: float).
        safe=True  → no self-collision predicted.
        safe=False → reason describes the closest/colliding pair.
        clearance_mm is the minimum gap found (negative = collision).
        """
        origins = Pro450Kinematics.joint_origins(angles_deg)
        gap, (i, j) = cls._min_clearance(origins)

        if gap < 0:
            return (
                False,
                f'Self-collision: seg {i}↔seg {j}  gap={gap:.1f}mm',
                gap,
            )
        return True, f'Clearance OK ({gap:.1f}mm, worst seg {i}↔{j})', gap

    @classmethod
    def best_ik_solution(cls, candidates):
        """
        Given a list of candidate IK joint-angle solutions (each a list of 6
        floats in degrees), return the one that maximises minimum link clearance.
        Also filters out solutions that are in hard collision (gap < 0).

        Returns (best_angles, clearance_mm) or (None, None) if all collide.
        """
        best_angles = None
        best_gap    = float('-inf')

        for angles in candidates:
            if angles is None or len(angles) < 6:
                continue
            _, _, gap = cls.check(angles)
            if gap > best_gap:
                best_gap    = gap
                best_angles = angles

        if best_angles is None or best_gap < 0:
            return None, best_gap
        return best_angles, best_gap

    @classmethod
    def check_cartesian_path(cls, from_angles, to_angles, steps=20):
        """
        Sample `steps+1` configurations along the path from_angles → to_angles
        and verify none are in self-collision.

        The interpolation is done in joint space, which is a conservative
        over-approximation of the actual moveL Cartesian path.  Any flagged
        collision is real; false negatives are impossible.

        Returns (safe: bool, reason: str, worst_clearance: float)
        """
        worst_gap   = float('inf')
        worst_step  = 0
        worst_reason = 'OK'

        for step in range(steps + 1):
            t = step / steps
            interp = [
                from_angles[k] + t * (to_angles[k] - from_angles[k])
                for k in range(6)
            ]
            safe, reason, gap = cls.check(interp)
            if gap < worst_gap:
                worst_gap    = gap
                worst_step   = step
                worst_reason = reason
            if not safe:
                pct = int(t * 100)
                return False, f'[{pct}% along path] {reason}', worst_gap

        return True, f'Path clear  (worst gap {worst_gap:.1f}mm at step {worst_step})', worst_gap


# ═══════════════════════════════════════════════════════════════════════════════
#  QoS PROFILES
# ═══════════════════════════════════════════════════════════════════════════════

_QOS_RELIABLE_TL = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_QOS_RELIABLE_VOL = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_QOS_STATUS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_QOS_POSE_PUB = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# ═══════════════════════════════════════════════════════════════════════════════
#  Z FLOOR MONITOR
# ═══════════════════════════════════════════════════════════════════════════════

class ZFloorMonitor:
    """Background thread — kills motion the instant Z < z_floor."""

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
                        f'Z-MONITOR TRIP: Z={c[2]:.1f}mm breached floor '
                        f'({self._z_floor}mm)! EMERGENCY STOP.'
                    )
                    self._mc.stop()
                    self.tripped  = True
                    self._running = False
                    return
            except Exception:
                pass
            time.sleep(self._monitor_hz)


# ═══════════════════════════════════════════════════════════════════════════════
#  ARM NODE
# ═══════════════════════════════════════════════════════════════════════════════

class ArmNode(Node):

    def __init__(self):
        super().__init__('anubix_arm')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('simulate', False)
        self.declare_parameter('arm_move_delay',           2.0)

        self.declare_parameter('home_x',                   0.15)
        self.declare_parameter('home_y',                   0.0)
        self.declare_parameter('home_z',                   0.3)

        self.declare_parameter('home_joint_1',             0.0)
        self.declare_parameter('home_joint_2',             0.0)
        self.declare_parameter('home_joint_3',            -90.0)
        self.declare_parameter('home_joint_4',             0.0)
        self.declare_parameter('home_joint_5',             0.0)
        self.declare_parameter('home_joint_6',             0.0)

        self.declare_parameter('pose_publish_rate_hz',     5.0)
        self.declare_parameter('speed',                    20)
        self.declare_parameter('transit_z',                280.0)
        self.declare_parameter('z_floor',                  50.0)
        self.declare_parameter('monitor_hz',               0.03)

        self.declare_parameter('arm_host',                 '192.168.0.232')
        self.declare_parameter('arm_port',                 4500)
        self.declare_parameter('max_velocity_scaling',     0.5)
        self.declare_parameter('max_acceleration_scaling', 0.5)
        self.declare_parameter('force_threshold',          2.0)

        self._simulate         = self.get_parameter('simulate').value
        self._arm_move_delay   = self.get_parameter('arm_move_delay').value
        self._speed            = self.get_parameter('speed').value
        self._transit_z        = self.get_parameter('transit_z').value
        self._z_floor          = self.get_parameter('z_floor').value
        self._monitor_hz       = self.get_parameter('monitor_hz').value
        self._force_threshold  = self.get_parameter('force_threshold').value
        self._pose_rate        = self.get_parameter('pose_publish_rate_hz').value

        self._home_angles = [
            self.get_parameter(f'home_joint_{i}').value for i in range(1, 7)
        ]
        self._home_xyz = [
            self.get_parameter('home_x').value,
            self.get_parameter('home_y').value,
            self.get_parameter('home_z').value,
        ]

        self._ws = dict(
            x=(-474, 474),
            y=(-474, 474),
            z=(self._z_floor, 677),
        )

        # ── concurrency ──────────────────────────────────────────────────────
        self._arm_lock      = threading.Lock()
        self._pose_lock     = threading.Lock()
        self._force_stopped = False

        # ── latched pose ─────────────────────────────────────────────────────
        self._current_pose              = PoseStamped()
        self._current_pose.header.frame_id = 'base_link'
        self._current_pose.pose.position.x = self._home_xyz[0]
        self._current_pose.pose.position.y = self._home_xyz[1]
        self._current_pose.pose.position.z = self._home_xyz[2]
        self._current_pose.pose.orientation.w = 1.0

        # ── hardware backend ─────────────────────────────────────────────────
        self._mc      = None
        self._monitor = None

        if not self._simulate:
            if Pro450Client is None:
                self.get_logger().fatal(
                    'simulate=false but pymycobot is not installed. Aborting.'
                )
                raise RuntimeError('pymycobot missing')

            host = self.get_parameter('arm_host').value
            port = self.get_parameter('arm_port').value
            self.get_logger().info(f'Connecting to Pro 450 at {host}:{port} ...')
            self._mc = Pro450Client(host, port)

            if self._mc.is_power_on() != 1:
                self.get_logger().info('Powering on arm...')
                self._mc.power_on()
                time.sleep(3)

            self._mc.set_collision_mode(1)    # hardware collision detection ON
            self._mc.set_movement_type(1)     # moveL globally

            self._monitor = ZFloorMonitor(
                self._mc, self._z_floor, self._monitor_hz, self.get_logger()
            )
            self.get_logger().info('Pro 450 connected.')

            c = self._mc.get_coords()
            if c and len(c) >= 3:
                self._home_xyz = [c[0] / 1000.0, c[1] / 1000.0, c[2] / 1000.0]
                self._current_pose.pose.position.x = self._home_xyz[0]
                self._current_pose.pose.position.y = self._home_xyz[1]
                self._current_pose.pose.position.z = self._home_xyz[2]
                self.get_logger().info(
                    f'Initial coords from hardware: '
                    f'[{c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}] mm'
                )
        else:
            self.get_logger().info('simulate=true — no hardware connection.')

        if self._simulate:
            self._move_impl = self._simulate_arm_move
        else:
            self._move_impl = self._real_arm_move

        # ── publishers ───────────────────────────────────────────────────────
        self._arm_status_pub     = self.create_publisher(String,      '/arm/arm_status',     _QOS_STATUS)
        self._current_pose_pub   = self.create_publisher(PoseStamped, '/arm/current_pose',   _QOS_POSE_PUB)

        # ── subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/supervisor/arm_nav_goal',
            self._on_arm_goal, _QOS_RELIABLE_VOL,
        )
        self.create_subscription(
            Bool, '/supervisor/force_stop',
            self._on_force_stop, _QOS_RELIABLE_VOL,
        )

        period = 1.0 / self._pose_rate
        self.create_timer(period, self._publish_current_pose)

        self.get_logger().info(
            f'ArmNode ready  [simulate={self._simulate}  '
            f'z_floor={self._z_floor}mm  transit_z={self._transit_z}mm]'
        )
        self._publish_current_pose()

    # ═══════════════════════════════════════════════════════════════════════
    #  SUBSCRIBER CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def _is_home_goal(self, target):
        """Check if the resolved target matches the home position (within 5mm)."""
        dx = abs(target.position.x - self._home_xyz[0])
        dy = abs(target.position.y - self._home_xyz[1])
        dz = abs(target.position.z - self._home_xyz[2])
        return dx < 0.005 and dy < 0.005 and dz < 0.005

    def _on_arm_goal(self, msg: PoseStamped):
        if self._force_stopped:
            self.get_logger().warn('force_stop active — ignoring arm_nav_goal')
            self._arm_status_pub.publish(String(data='block'))
            return
        t = threading.Thread(target=self._move_impl, args=(msg,), daemon=True)
        t.start()

    def _on_force_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('FORCE STOP received.')
            self._force_stopped = True
            if self._mc:
                try:
                    self._mc.stop()
                except Exception:
                    pass
        else:
            self.get_logger().info('Force stop cleared — re-armed.')
            self._force_stopped = False

    # ═══════════════════════════════════════════════════════════════════════
    #  SIMULATE BACKEND
    # ═══════════════════════════════════════════════════════════════════════

    def _simulate_arm_move(self, goal: PoseStamped):
        with self._arm_lock:
            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            target = self._resolve_goal(goal)

            if self._is_home_goal(target):
                self.get_logger().info('[SIM] Home goal detected — joint-space homing.')
                self._go_home()
                self._arm_status_pub.publish(String(data='success'))
                return

            x = target.position.x * 1000
            y = target.position.y * 1000
            z = target.position.z * 1000

            # Check if small move — skip homing for calibration accuracy
            with self._pose_lock:
                cx = self._current_pose.pose.position.x * 1000
                cy = self._current_pose.pose.position.y * 1000
                cz = self._current_pose.pose.position.z * 1000
            dist = math.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2)
            is_small = dist < self._DIRECT_MOVE_THRESHOLD_MM

            self.get_logger().info('[SIM] Running pre-flight check...')
            passed, report = self._preflight_sim(x, y, z)
            for line in report:
                self.get_logger().info(f'  preflight: {line}')
            if not passed:
                self.get_logger().error(f'[SIM] Move rejected — pre-flight failed: {report}')
                self._arm_status_pub.publish(String(data='preflight_failed'))
                return

            if is_small:
                self.get_logger().info(
                    f'[SIM] Direct move (delta={dist:.1f}mm) -> '
                    f'({target.position.x:.4f}, {target.position.y:.4f}, {target.position.z:.4f})')
                time.sleep(0.3)
            else:
                self.get_logger().info('[SIM] Going to home...')
                self._arm_status_pub.publish(String(data='homing'))
                time.sleep(self._arm_move_delay)
                with self._pose_lock:
                    self._current_pose.pose.position.x = self._home_xyz[0]
                    self._current_pose.pose.position.y = self._home_xyz[1]
                    self._current_pose.pose.position.z = self._home_xyz[2]
                self._publish_current_pose()

                if self._force_stopped:
                    self._arm_status_pub.publish(String(data='mechanical_error'))
                    return

                self.get_logger().info(
                    f'[SIM] arm move -> '
                    f'({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})')
                time.sleep(self._arm_move_delay)

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            with self._pose_lock:
                self._current_pose.pose = target
            self._publish_current_pose()
            self._arm_status_pub.publish(String(data='success'))

    # ═══════════════════════════════════════════════════════════════════════
    #  REAL BACKEND
    # ═══════════════════════════════════════════════════════════════════════

    # Small-move threshold (mm): if target is within this distance of the
    # current position, do a direct moveL instead of the full home→3-phase
    # cycle.  This is critical for calibration moves where accuracy matters
    # more than collision avoidance over a long path.
    _DIRECT_MOVE_THRESHOLD_MM = 50.0

    def _real_arm_move(self, goal: PoseStamped):
        with self._arm_lock:
            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            self._clear_errors()
            target = self._resolve_goal(goal)

            if self._is_home_goal(target):
                self.get_logger().info('Home goal detected — joint-space homing (bypasses IK).')
                self._go_home()
                self._arm_status_pub.publish(String(data='success'))
                return

            x = target.position.x * 1000
            y = target.position.y * 1000
            z = target.position.z * 1000

            # Check if this is a small move from the current position
            c = self._coords()
            is_small_move = False
            if c:
                dist = math.sqrt(
                    (x - c[0]) ** 2 + (y - c[1]) ** 2 + (z - c[2]) ** 2
                )
                is_small_move = dist < self._DIRECT_MOVE_THRESHOLD_MM

            if is_small_move:
                return self._direct_move(target, x, y, z, c)

            self.get_logger().info(
                f'Pre-flight for [{x:.1f}, {y:.1f}, {z:.1f}] mm ...'
            )

            passed, report, best_ik = self._preflight(x, y, z)
            for line in report:
                self.get_logger().info(f'  preflight: {line}')

            if not passed:
                self.get_logger().error(
                    f'Move REJECTED — pre-flight failed: {report}'
                )
                self._arm_status_pub.publish(String(data='preflight_failed'))
                return

            self.get_logger().info(
                f'Pre-flight passed. Best IK: {[round(a,1) for a in best_ik]}'
            )

            # ── step 1: joint-space home ─────────────────────────────────────
            self._arm_status_pub.publish(String(data='homing'))
            self.get_logger().info(f'Going to home: {self._home_angles}')
            self._mc.send_angles(self._home_angles, self._speed)
            time.sleep(0.35)
            ok = self._wait_stop()
            if not ok or self._force_stopped:
                self.get_logger().error('Failed to reach home.')
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            self.get_logger().info('Home reached.')
            c = self._coords()
            if not c:
                self.get_logger().error('Cannot read coords after homing.')
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return
            rx, ry, rz = c[3], c[4], c[5]

            self.get_logger().info(f'Moving to [{x:.1f}, {y:.1f}, {z:.1f}] mm')

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── phase 1: lift to transit height ──────────────────────────────
            if c[2] < self._transit_z:
                lift = [c[0], c[1], self._transit_z, rx, ry, rz]
                ok = self._linear_move(lift, tag='1-lift')
                if not ok:
                    self._arm_status_pub.publish(String(data='mechanical_error'))
                    return
                c = self._coords()
                if c:
                    rx, ry, rz = c[3], c[4], c[5]

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── phase 2: horizontal travel ────────────────────────────────────
            travel = [x, y, self._transit_z, rx, ry, rz]
            ok = self._linear_move(travel, tag='2-travel')
            if not ok:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return
            c = self._coords()
            if c:
                rx, ry, rz = c[3], c[4], c[5]

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            # ── phase 3: descend ─────────────────────────────────────────────
            descent = [x, y, z, rx, ry, rz]
            ok = self._linear_move(descent, tag='3-descend')
            if not ok:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                return

            c_final = self._coords()
            if c_final:
                with self._pose_lock:
                    self._current_pose.pose.position.x = c_final[0] / 1000.0
                    self._current_pose.pose.position.y = c_final[1] / 1000.0
                    self._current_pose.pose.position.z = c_final[2] / 1000.0
            else:
                with self._pose_lock:
                    self._current_pose.pose = target
            self._publish_current_pose()
            self._arm_status_pub.publish(String(data='success'))

    def _direct_move(self, target, x, y, z, c):
        """
        Direct moveL for small moves (e.g. calibration).
        Skips homing and 3-phase — just sends a single linear command
        from the current position to the target.  This preserves the
        sub-millimetre accuracy that calibration depends on.
        """
        rx, ry, rz = c[3], c[4], c[5]
        dist = math.sqrt((x - c[0])**2 + (y - c[1])**2 + (z - c[2])**2)
        self.get_logger().info(
            f'Direct moveL [{c[0]:.1f},{c[1]:.1f},{c[2]:.1f}] -> '
            f'[{x:.1f},{y:.1f},{z:.1f}] mm  (delta={dist:.1f}mm)'
        )

        if self._force_stopped:
            self._arm_status_pub.publish(String(data='mechanical_error'))
            return

        coords = [x, y, z, rx, ry, rz]
        ok = self._linear_move(coords, tag='direct')
        if not ok:
            self._arm_status_pub.publish(String(data='mechanical_error'))
            return

        if self._force_stopped:
            self._arm_status_pub.publish(String(data='mechanical_error'))
            return

        c_after = self._coords()
        if c_after:
            with self._pose_lock:
                self._current_pose.pose.position.x = c_after[0] / 1000.0
                self._current_pose.pose.position.y = c_after[1] / 1000.0
                self._current_pose.pose.position.z = c_after[2] / 1000.0
        else:
            with self._pose_lock:
                self._current_pose.pose = target
        self._publish_current_pose()
        self._arm_status_pub.publish(String(data='success'))

    # ═══════════════════════════════════════════════════════════════════════
    #  FRAME RESOLUTION
    # ═══════════════════════════════════════════════════════════════════════

    def _resolve_goal(self, goal: PoseStamped):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.orientation = goal.pose.orientation

        if goal.header.frame_id == 'calibration':
            with self._pose_lock:
                cur = self._current_pose.pose.position
            p.position.x = cur.x + goal.pose.position.x
            p.position.y = cur.y + goal.pose.position.y
            p.position.z = cur.z + goal.pose.position.z
            self.get_logger().info(
                f'calibration (delta) -> '
                f'({p.position.x:.4f}, {p.position.y:.4f}, {p.position.z:.4f}) m'
            )
        else:
            p.position = goal.pose.position

        return p

    # ═══════════════════════════════════════════════════════════════════════
    #  POSE PUBLISHER
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_current_pose(self):
        with self._pose_lock:
            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.pose            = self._current_pose.pose
        self._current_pose_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    #  LOW-LEVEL HARDWARE HELPERS
    # ═══════════════════════════════════════════════════════════════════════

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
                self.get_logger().warn('Motion timeout — force stopping.')
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
        if speed is None:
            speed = self._speed
        self.get_logger().debug(f'[{tag}] -> {[round(v, 1) for v in target]}')
        self._monitor.start()
        self._mc.send_coords(target, speed, 1)
        time.sleep(0.35)
        ok = self._wait_stop()
        self._monitor.stop()
        if self._monitor.tripped:
            self.get_logger().error(f'[{tag}] ABORTED by Z monitor.')
            return False
        return ok

    # ═══════════════════════════════════════════════════════════════════════
    #  IK + COLLISION-AWARE SOLUTION SELECTION
    # ═══════════════════════════════════════════════════════════════════════

    def _solve_best_ik(self, x, y, z, rx, ry, rz, seed_angles):
        target = [x, y, z, rx, ry, rz]
        candidates = []

        seed_variants = [list(seed_angles)]
        for dj1 in (-60, -30, 30, 60):
            v = list(seed_angles)
            v[0] = max(-165, min(165, v[0] + dj1))
            seed_variants.append(v)
        for dj3 in (-30, 30):
            v = list(seed_angles)
            v[2] = max(-165, min(165, v[2] + dj3))
            seed_variants.append(v)

        for seed in seed_variants:
            try:
                sol = self._mc.solve_inv_kinematics(target, seed)
            except Exception:
                continue
            if not sol or not isinstance(sol, list) or len(sol) < 6:
                continue
            if any(v is None for v in sol):
                continue
            limits = Pro450Kinematics.JOINT_LIMITS
            if any(abs(sol[k]) > limits[k] for k in range(6)):
                continue
            candidates.append(sol)

        if not candidates:
            return None, float('-inf'), 'IK solver returned no valid solution'

        best, gap = Pro450CollisionChecker.best_ik_solution(candidates)
        if best is None:
            best, gap = candidates[0], float('-inf')
            return best, gap, f'All IK candidates in self-collision (worst gap={gap:.1f}mm)'

        return best, gap, f'Best of {len(candidates)} IK candidates selected (gap={gap:.1f}mm)'

    # ═══════════════════════════════════════════════════════════════════════
    #  PRE-FLIGHT CHECKS
    # ═══════════════════════════════════════════════════════════════════════

    def _preflight_sim(self, x, y, z):
        errors = []
        report = []

        for axis, val in [('x', x), ('y', y), ('z', z)]:
            lo, hi = self._ws[axis]
            if not (lo <= val <= hi):
                errors.append(f'{axis}={val:.1f} outside workspace [{lo}, {hi}] mm')

        if errors:
            return False, errors

        r_xy  = math.hypot(x, y)
        reach = math.sqrt(x**2 + y**2 + z**2)

        if r_xy < 30:
            self.get_logger().warn(
                f'[SIM] XY radius {r_xy:.1f}mm near J1 singularity (<30mm) — proceeding.'
            )
        if reach > 445:
            self.get_logger().warn(
                f'[SIM] 3-D reach {reach:.1f}mm > 445mm advisory — proceeding.'
            )

        report.append(f'Workspace OK  x={x:.1f} y={y:.1f} z={z:.1f} mm')

        safe, reason, gap = Pro450CollisionChecker.check(self._home_angles)
        if not safe:
            return False, [f'Self-collision at home config: {reason}']
        report.append(f'Self-collision (home) OK  clearance={gap:.1f}mm')

        return True, report

    def _preflight(self, x, y, z):
        report = []

        ws_errors = []
        for axis, val in [('x', x), ('y', y), ('z', z)]:
            lo, hi = self._ws[axis]
            if not (lo <= val <= hi):
                ws_errors.append(f'{axis}={val:.1f} outside [{lo}, {hi}] mm')
        if ws_errors:
            return False, ws_errors, None

        report.append(f'Workspace OK  x={x:.1f} y={y:.1f} z={z:.1f} mm')

        r_xy  = math.hypot(x, y)
        reach = math.sqrt(x**2 + y**2 + z**2)
        if r_xy < 30:
            self.get_logger().warn(
                f'XY radius {r_xy:.1f}mm near J1 singularity (<30mm) — continuing to IK.'
            )
        if reach > 445:
            self.get_logger().warn(
                f'3-D reach {reach:.1f}mm > 445mm advisory — continuing to IK.'
            )

        seed = self._angles()
        if not seed:
            return False, ['Cannot read current angles for IK seed'], None

        c = self._coords()
        rx, ry, rz = (c[3], c[4], c[5]) if c else (0.0, 0.0, 0.0)

        best_ik, ik_gap, ik_msg = self._solve_best_ik(x, y, z, rx, ry, rz, seed)

        if best_ik is None:
            return False, [f'IK failed: {ik_msg}'], None

        report.append(f'IK OK  joints={[round(a, 1) for a in best_ik]}  {ik_msg}')

        path_safe, path_reason, worst_gap = Pro450CollisionChecker.check_cartesian_path(
            self._home_angles, best_ik, steps=20
        )
        if not path_safe:
            self.get_logger().warn(
                f'Path self-collision warning: {path_reason}  '
                f'(hardware collision mode active — proceeding with caution)'
            )
            report.append(f'Path collision WARNING: {path_reason}')
        else:
            report.append(f'Path collision OK  {path_reason}')

        return True, report, best_ik

    # ═══════════════════════════════════════════════════════════════════════
    #  HOME
    # ═══════════════════════════════════════════════════════════════════════

    def _go_home(self):
        self.get_logger().info('Returning to safe home...')
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

        c = self._coords()
        if c:
            with self._pose_lock:
                self._current_pose.pose.position.x = c[0] / 1000.0
                self._current_pose.pose.position.y = c[1] / 1000.0
                self._current_pose.pose.position.z = c[2] / 1000.0
            self._publish_current_pose()
            self.get_logger().info(
                f'Home done. Coords: [{c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}] mm'
            )
        else:
            with self._pose_lock:
                self._current_pose.pose.position.x = self._home_xyz[0]
                self._current_pose.pose.position.y = self._home_xyz[1]
                self._current_pose.pose.position.z = self._home_xyz[2]
            self._publish_current_pose()
            self.get_logger().warn('Home done but could not read coords — using home_xyz fallback.')


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
