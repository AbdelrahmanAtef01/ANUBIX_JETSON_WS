#!/usr/bin/env python3
"""
ANUBIX Jetson Bridge Node
==========================
Runs on the Jetson Orin Nano.

Responsibilities:
  1. Publish 1 Hz heartbeat so the RPi knows the Jetson is alive.
  2. Monitor RPi heartbeat; log ERROR if silent.
  3. Log every supervisor command and feedback for traceability.
  4. Publish /bridge/connection_status at 1 Hz.
"""

import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose


class JetsonBridgeNode(Node):

    def __init__(self):
        super().__init__('anubix_jetson_bridge')

        self.declare_parameter('rpi_ip', '192.168.10.2')
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('connection_timeout', 5.0)

        self._rpi_ip = self.get_parameter('rpi_ip').value
        self._hb_interval = self.get_parameter('heartbeat_interval').value
        self._conn_timeout = self.get_parameter('connection_timeout').value

        self._last_rpi_hb: float = 0.0
        self._rpi_connected: bool = False
        self._seq: int = 0
        self._lock = threading.Lock()

        self._stats = {
            'nav_goals_dispatched': 0,
            'nav_vision_flags': 0,
            'perception_goals_dispatched': 0,
            'camera_switches': 0,
            'force_stops': 0,
            'nav_feedbacks': 0,
            'perception_feedbacks': 0,
            'target_poses': 0,
        }

        self._sub_group = ReentrantCallbackGroup()

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
        # force_stop is edge-triggered — VOLATILE so a stale latched
        # True from a previous session never reaches this observer.
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers
        self._hb_pub = self.create_publisher(
            String, '/bridge/jetson_heartbeat', reliable_qos)
        self._conn_pub = self.create_publisher(
            String, '/bridge/connection_status', reliable_qos)

        # Subscriptions
        self.create_subscription(
            String, '/bridge/rpi_heartbeat', self._on_rpi_heartbeat,
            reliable_qos, callback_group=self._sub_group)
        self.create_subscription(
            PoseStamped, '/supervisor/nav_goal', self._on_nav_goal,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/nav_vision', self._on_nav_vision,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/supervisor/perception_goal', self._on_perception_goal,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/supervisor/target_camera', self._on_target_camera,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop,
            force_stop_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/nav/status', self._on_nav_status,
            reliable_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/perception/status', self._on_perception_status,
            reliable_qos, callback_group=self._sub_group)
        self.create_subscription(
            Pose, '/perception/target_pose', self._on_target_pose,
            reliable_qos, callback_group=self._sub_group)

        # Timers
        self.create_timer(self._hb_interval, self._publish_heartbeat)
        self.create_timer(self._hb_interval, self._check_rpi_connection)

        self.get_logger().info('=' * 62)
        self.get_logger().info('  ANUBIX Jetson Bridge Node')
        self.get_logger().info(f'  Monitoring link to RPi @ {self._rpi_ip}')
        self.get_logger().info(
            f'  Heartbeat: {self._hb_interval}s  Timeout: {self._conn_timeout}s')
        self.get_logger().info('=' * 62)

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
                f'status={pl.get("status","?")}')
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
                f'Check: ethernet, RPi power, RPi bridge running, '
                f'same ROS_DOMAIN_ID.')
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

    def _on_nav_goal(self, msg: PoseStamped):
        with self._lock:
            self._stats['nav_goals_dispatched'] += 1
            total = self._stats['nav_goals_dispatched']
            rpi_ok = self._rpi_connected

        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(
            f'[BRIDGE->RPi] /supervisor/nav_goal pos=({x:.3f}, {y:.3f}) '
            f'[total={total}]')
        if not rpi_ok:
            self.get_logger().warning(
                '[BRIDGE->RPi] WARNING: RPi DISCONNECTED — nav may not receive it!')

    def _on_nav_vision(self, msg: Bool):
        with self._lock:
            self._stats['nav_vision_flags'] += 1
            total = self._stats['nav_vision_flags']

        vision_flag = bool(msg.data)
        self.get_logger().info(
            f'[BRIDGE->RPi] /supervisor/nav_vision vision={vision_flag} '
            f'[total={total}]')

    def _on_perception_goal(self, msg: String):
        with self._lock:
            self._stats['perception_goals_dispatched'] += 1
            total = self._stats['perception_goals_dispatched']

        self.get_logger().info(
            f'[BRIDGE->RPi] /supervisor/perception_goal task="{msg.data}" '
            f'[total={total}]')

    def _on_target_camera(self, msg: String):
        with self._lock:
            self._stats['camera_switches'] += 1
            total = self._stats['camera_switches']

        self.get_logger().info(
            f'[BRIDGE->RPi] /supervisor/target_camera camera={msg.data} '
            f'[total={total}]')

    def _on_force_stop(self, msg: Bool):
        # Log both edges: True is the abort event, False is the re-arm that
        # the master publishes immediately afterwards (or at startup to
        # clear a stale latched True).
        if msg.data:
            with self._lock:
                self._stats['force_stops'] += 1
                total = self._stats['force_stops']
            self.get_logger().warning(
                f'[BRIDGE] /supervisor/force_stop=TRUE [total={total}]')
        else:
            self.get_logger().info(
                '[BRIDGE] /supervisor/force_stop=False (re-armed)')

    def _on_nav_status(self, msg: String):
        with self._lock:
            self._stats['nav_feedbacks'] += 1
            total = self._stats['nav_feedbacks']
        status = msg.data.strip().lower()
        log_fn = (self.get_logger().warning
                  if status in ('blocked', 'failure')
                  else self.get_logger().info)
        log_fn(f'[BRIDGE<-RPi] /nav/status = "{status}" [total={total}]')

    def _on_perception_status(self, msg: String):
        with self._lock:
            self._stats['perception_feedbacks'] += 1
            total = self._stats['perception_feedbacks']
        self.get_logger().info(
            f'[BRIDGE<-RPi] /perception/status = "{msg.data.strip()}" '
            f'[total={total}]')

    def _on_target_pose(self, msg: Pose):
        with self._lock:
            self._stats['target_poses'] += 1
            total = self._stats['target_poses']
        self.get_logger().info(
            f'[BRIDGE<-RPi] /perception/target_pose '
            f'pos=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}) '
            f'[total={total}]')


def main(args=None):
    rclpy.init(args=args)
    node = JetsonBridgeNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('[BRIDGE] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
