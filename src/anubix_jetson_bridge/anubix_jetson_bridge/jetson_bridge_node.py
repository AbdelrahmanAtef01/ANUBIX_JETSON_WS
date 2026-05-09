#!/usr/bin/env python3
"""
ANUBIX Jetson Bridge Node
==========================
Runs on the Jetson Orin Nano.

Responsibilities:
  1. Publish a 1 Hz heartbeat so the RPi knows the Jetson is alive.
  2. Monitor the RPi heartbeat; log an ERROR if it goes silent.
  3. Sniff every supervisor command dispatched toward the RPi stacks
     (nav_goal, perception_goal, target_camera, force_stop) and log
     with full details so cross-machine traffic is fully traceable.
  4. Sniff every feedback arriving from RPi stacks (nav/status,
     perception/status, perception/target_pose) for the same reason.
  5. Publish /bridge/connection_status at 1 Hz so other nodes and
     operators can check link health at a glance.

Nothing is republished or modified — this node is purely observational
plus heartbeat bookkeeping.

Topics produced:
  /bridge/jetson_heartbeat   std_msgs/String  (JSON, 1 Hz)
  /bridge/connection_status  std_msgs/String  (JSON, 1 Hz)

Topics subscribed (read-only):
  /bridge/rpi_heartbeat        std_msgs/String
  /supervisor/nav_goal         geometry_msgs/PoseStamped
  /supervisor/perception_goal  std_msgs/String
  /supervisor/target_camera    std_msgs/String
  /supervisor/force_stop       std_msgs/Bool
  /nav/status                  std_msgs/String
  /perception/status           std_msgs/String
  /perception/target_pose      geometry_msgs/Pose
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose


class JetsonBridgeNode(Node):

    def __init__(self):
        super().__init__('anubix_jetson_bridge')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('rpi_ip', '192.168.10.2')
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('connection_timeout', 5.0)

        self._rpi_ip = self.get_parameter('rpi_ip').value
        self._hb_interval = self.get_parameter('heartbeat_interval').value
        self._conn_timeout = self.get_parameter('connection_timeout').value

        # ── State ─────────────────────────────────────────────────────────────
        self._last_rpi_hb: float = 0.0
        self._rpi_connected: bool = False
        self._seq: int = 0
        self._lock = threading.Lock()

        self._stats = {
            'nav_goals_dispatched': 0,
            'perception_goals_dispatched': 0,
            'camera_switches': 0,
            'force_stops': 0,
            'nav_feedbacks': 0,
            'perception_feedbacks': 0,
            'target_poses': 0,
        }

        # ── QoS profiles ──────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._hb_pub = self.create_publisher(
            String, '/bridge/jetson_heartbeat', reliable_qos)
        self._conn_pub = self.create_publisher(
            String, '/bridge/connection_status', reliable_qos)

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Heartbeat from RPi
        self.create_subscription(
            String, '/bridge/rpi_heartbeat', self._on_rpi_heartbeat, reliable_qos)

        # Cross-machine commands: Jetson → RPi
        self.create_subscription(
            PoseStamped, '/supervisor/nav_goal', self._on_nav_goal, cmd_qos)
        self.create_subscription(
            String, '/supervisor/perception_goal', self._on_perception_goal, cmd_qos)
        self.create_subscription(
            String, '/supervisor/target_camera', self._on_target_camera, cmd_qos)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop, cmd_qos)

        # Cross-machine feedback: RPi → Jetson
        self.create_subscription(
            String, '/nav/status', self._on_nav_status, reliable_qos)
        self.create_subscription(
            String, '/perception/status', self._on_perception_status, reliable_qos)
        self.create_subscription(
            Pose, '/perception/target_pose', self._on_target_pose, reliable_qos)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(self._hb_interval, self._publish_heartbeat)
        self.create_timer(self._hb_interval, self._check_rpi_connection)

        self.get_logger().info('=' * 62)
        self.get_logger().info('  ANUBIX Jetson Bridge Node')
        self.get_logger().info(f'  Monitoring link to RPi @ {self._rpi_ip}')
        self.get_logger().info(f'  Heartbeat: {self._hb_interval}s  Timeout: {self._conn_timeout}s')
        self.get_logger().info('=' * 62)

    # ── Heartbeat ──────────────────────────────────────────────────────────────

    def _publish_heartbeat(self):
        with self._lock:
            self._seq += 1
            seq = self._seq
            stats = dict(self._stats)
            rpi_ok = self._rpi_connected

        payload = json.dumps({
            'source': 'jetson',
            'seq': seq,
            'stamp': round(time.time(), 3),
            'status': 'ok',
            'rpi_connected': rpi_ok,
            'stats': stats,
        }, separators=(',', ':'))
        self._hb_pub.publish(String(data=payload))

    def _on_rpi_heartbeat(self, msg: String):
        now = time.time()
        with self._lock:
            self._last_rpi_hb = now
        try:
            pl = json.loads(msg.data)
            self.get_logger().debug(
                f'[BRIDGE] RPi heartbeat seq={pl.get("seq","?")} '
                f'status={pl.get("status","?")} '
                f'jetson_connected={pl.get("jetson_connected","?")}')
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(
                f'[BRIDGE] Malformed RPi heartbeat: {msg.data!r}')

    def _check_rpi_connection(self):
        with self._lock:
            last = self._last_rpi_hb
            was_ok = self._rpi_connected

        elapsed = time.time() - last if last > 0 else float('inf')
        now_ok = last > 0 and elapsed < self._conn_timeout

        with self._lock:
            self._rpi_connected = now_ok

        if was_ok and not now_ok:
            self.get_logger().error(
                f'[BRIDGE] *** RPi CONNECTION LOST *** '
                f'No heartbeat for {elapsed:.1f}s '
                f'(timeout={self._conn_timeout}s). '
                f'Check: ethernet cable, RPi power, RPi bridge node running.')
        elif not was_ok and now_ok:
            self.get_logger().info(
                f'[BRIDGE] RPi connection ESTABLISHED '
                f'(heartbeat age={elapsed:.2f}s)')

        conn_payload = json.dumps({
            'source': 'jetson',
            'stamp': round(time.time(), 3),
            'rpi_connected': now_ok,
            'rpi_heartbeat_age_s': round(elapsed, 2) if last > 0 else None,
        }, separators=(',', ':'))
        self._conn_pub.publish(String(data=conn_payload))

    # ── Command monitors: Jetson → RPi ─────────────────────────────────────────

    def _on_nav_goal(self, msg: PoseStamped):
        with self._lock:
            self._stats['nav_goals_dispatched'] += 1
            total = self._stats['nav_goals_dispatched']
            rpi_ok = self._rpi_connected

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        frame = msg.header.frame_id

        self.get_logger().info(
            f'[BRIDGE→RPi] /supervisor/nav_goal  '
            f'pos=({x:.3f}, {y:.3f}, {z:.3f})  frame={frame!r}  '
            f'[dispatched total={total}]')

        if not rpi_ok:
            self.get_logger().warning(
                '[BRIDGE→RPi] WARNING: nav_goal dispatched but RPi appears DISCONNECTED — '
                'navigation stack may not receive it')

    def _on_perception_goal(self, msg: String):
        with self._lock:
            self._stats['perception_goals_dispatched'] += 1
            total = self._stats['perception_goals_dispatched']
            rpi_ok = self._rpi_connected

        self.get_logger().info(
            f'[BRIDGE→RPi] /supervisor/perception_goal  '
            f'task={msg.data!r}  [dispatched total={total}]')

        if not rpi_ok:
            self.get_logger().warning(
                '[BRIDGE→RPi] WARNING: perception_goal dispatched but RPi appears DISCONNECTED')

    def _on_target_camera(self, msg: String):
        with self._lock:
            self._stats['camera_switches'] += 1
            total = self._stats['camera_switches']

        self.get_logger().info(
            f'[BRIDGE→RPi] /supervisor/target_camera  '
            f'camera={msg.data}  [total switches={total}]')

    def _on_force_stop(self, msg: Bool):
        if not msg.data:
            return
        with self._lock:
            self._stats['force_stops'] += 1
            total = self._stats['force_stops']

        self.get_logger().warning(
            f'[BRIDGE] /supervisor/force_stop TRUE — '
            f'propagating to ALL nodes on both machines  [total={total}]')

    # ── Feedback monitors: RPi → Jetson ────────────────────────────────────────

    def _on_nav_status(self, msg: String):
        with self._lock:
            self._stats['nav_feedbacks'] += 1
            total = self._stats['nav_feedbacks']

        status = msg.data.strip().lower()
        log = (self.get_logger().warning
               if status in ('blocked', 'failure')
               else self.get_logger().info)
        log(f'[BRIDGE←RPi] /nav/status = {status!r}  [total feedbacks={total}]')

    def _on_perception_status(self, msg: String):
        with self._lock:
            self._stats['perception_feedbacks'] += 1
            total = self._stats['perception_feedbacks']

        self.get_logger().info(
            f'[BRIDGE←RPi] /perception/status = {msg.data.strip()!r}  '
            f'[total feedbacks={total}]')

    def _on_target_pose(self, msg: Pose):
        with self._lock:
            self._stats['target_poses'] += 1
            total = self._stats['target_poses']

        self.get_logger().info(
            f'[BRIDGE←RPi] /perception/target_pose  '
            f'pos=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})  '
            f'[total received={total}]')


def main(args=None):
    rclpy.init(args=args)
    node = JetsonBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[BRIDGE] Jetson bridge node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
