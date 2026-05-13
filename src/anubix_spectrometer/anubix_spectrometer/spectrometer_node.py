#!/usr/bin/env python3
"""
ANUBIX Spectrometer Stack - ROS 2 Node (Jetson Orin Nano)
==========================================================
Subscribes to: /supervisor/spectral_target (std_msgs/String)
               /supervisor/force_stop (std_msgs/Bool)
Publishes to:  /spectrometer/status (std_msgs/String)
               /spectrometer/result (std_msgs/String)  JSON — on success only

Status values: reading | applying_ML | uploading | success | failure

The /supervisor/spectral_target payload can be either:
  - "task_type"                              (legacy form)
  - "task_type|robot_id|task_id"             (preferred — propagates the
    OmniLink-supplied UUIDs through to Supabase)
"""

import os
import json
import time
import threading
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool

try:
    from ament_index_python.packages import get_package_share_directory
    AMENT_AVAILABLE = True
except ImportError:
    AMENT_AVAILABLE = False

from anubix_spectrometer.spectrometer_driver import (
    SpectrometerPipeline,
    SpectrometerDevice,
)


class SpectrometerNode(Node):

    def __init__(self):
        super().__init__('anubix_spectrometer')

        # Parameters
        # The spectrometer presents itself as a USB-Ethernet gadget — the
        # Jetson talks to it over TCP/IP, NOT serial. The sensor exposes
        # two ports: read_port (data in) and write_port (commands out).
        self.declare_parameter('host', '192.168.137.2')
        self.declare_parameter('read_port', 5000)
        self.declare_parameter('write_port', 5001)
        self.declare_parameter('num_channels', 257)
        self.declare_parameter('integration_time_ms', 100)
        self.declare_parameter('connect_timeout_s', 5.0)
        self.declare_parameter('read_timeout_s', 5.0)
        self.declare_parameter('bg_file', '')

        self._host = self.get_parameter('host').value
        self._read_port = int(self.get_parameter('read_port').value)
        self._write_port = int(self.get_parameter('write_port').value)
        self._num_channels = self.get_parameter('num_channels').value
        self._integration_time_ms = self.get_parameter('integration_time_ms').value
        self._connect_timeout_s = float(self.get_parameter('connect_timeout_s').value)
        self._read_timeout_s = float(self.get_parameter('read_timeout_s').value)
        bg_file_param = self.get_parameter('bg_file').value

        # Locate bg.csv
        if bg_file_param:
            self._bg_path = bg_file_param
        elif AMENT_AVAILABLE:
            try:
                pkg_share = get_package_share_directory('anubix_spectrometer')
                self._bg_path = os.path.join(pkg_share, 'config', 'bg.csv')
            except Exception:
                self._bg_path = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'bg.csv')
        else:
            self._bg_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'bg.csv')

        if not os.path.exists(self._bg_path):
            self.get_logger().fatal(
                f'[SPECTRO] Background file NOT FOUND: {self._bg_path}\n'
                f'  Set the bg_file parameter to the correct path.')
            raise FileNotFoundError(self._bg_path)

        self._sub_group = ReentrantCallbackGroup()

        # QoS profiles
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.create_subscription(
            String, '/supervisor/spectral_target', self._on_spectral_target,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop,
            cmd_qos, callback_group=self._sub_group)

        # Publishers
        self._status_pub = self.create_publisher(
            String, '/spectrometer/status', pub_qos)
        self._result_pub = self.create_publisher(
            String, '/spectrometer/result', pub_qos)

        # Initialize pipeline
        device = SpectrometerDevice(
            host=self._host,
            read_port=self._read_port,
            write_port=self._write_port,
            num_channels=self._num_channels,
            integration_time_ms=self._integration_time_ms,
            connect_timeout_s=self._connect_timeout_s,
            read_timeout_s=self._read_timeout_s,
        )

        self._pipeline = SpectrometerPipeline(
            bg_path=self._bg_path,
            device=device,
        )
        self._pipeline.set_status_callback(self._publish_status)

        try:
            self._pipeline.connect()
            self.get_logger().info('[SPECTRO] Pipeline connected to device')
        except Exception as e:
            self.get_logger().error(
                f'[SPECTRO] Failed to connect pipeline: {e}\n'
                f'{traceback.format_exc()}')

        self._busy = False
        self._force_stopped = False
        self._lock = threading.Lock()

        self.get_logger().info('=' * 50)
        self.get_logger().info('  ANUBIX Spectrometer Node - Jetson Orin Nano')
        self.get_logger().info(
            f'  Host: TCP {self._host} '
            f'(read={self._read_port}, write={self._write_port})')
        self.get_logger().info(f'  Channels: {self._num_channels}')
        self.get_logger().info(f'  BG file: {self._bg_path}')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            '[SPECTRO] Subscribed to /supervisor/spectral_target (String)')
        self.get_logger().info(
            '[SPECTRO] Publishing on /spectrometer/status, /spectrometer/result')
        self.get_logger().info('[SPECTRO] Ready and waiting for targets.')

    def _publish_status(self, status: str):
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f'[SPECTRO] Status published: "{status}"')

    def _on_force_stop(self, msg: Bool):
        if msg.data:
            self._force_stopped = True
            self.get_logger().warning(
                '[SPECTRO] *** FORCE STOP RECEIVED ***')

    def _on_spectral_target(self, msg: String):
        raw = msg.data.strip()
        parts = raw.split('|')
        task_type = parts[0].strip().lower() if parts else ''
        robot_id = parts[1].strip() if len(parts) > 1 else ''
        task_id = parts[2].strip() if len(parts) > 2 else ''

        self.get_logger().info(
            f'[SPECTRO] ========================================')
        self.get_logger().info(
            f'[SPECTRO] Target RECEIVED: task="{task_type}" '
            f'robot_id="{robot_id or "(none)"}" '
            f'task_id="{task_id or "(none)"}"')
        self.get_logger().info(
            f'[SPECTRO] ========================================')

        if self._force_stopped:
            self.get_logger().warning(
                '[SPECTRO] REJECTED: force_stopped. Publishing "failure".')
            self._publish_status('failure')
            return

        with self._lock:
            if self._busy:
                self.get_logger().warning(
                    '[SPECTRO] Already processing! Ignoring duplicate request.')
                return
            self._busy = True

        threading.Thread(
            target=self._run_pipeline,
            args=(task_type, robot_id, task_id),
            daemon=True,
        ).start()

    def _run_pipeline(self, task_type: str, robot_id: str = '', task_id: str = ''):
        try:
            self.get_logger().info(
                f'[SPECTRO] Starting pipeline for task: "{task_type}"')
            status, result = self._pipeline.run(task_type)

            if result:
                self.get_logger().info(
                    f'[SPECTRO] Analysis complete: '
                    f'classification="{result.classification}" '
                    f'confidence={result.confidence:.2%} '
                    f'value={result.value:.4f}')
                self.get_logger().info(
                    f'[SPECTRO] Details: {result.details}')

                result_payload = json.dumps({
                    'task_type': result.task_type,
                    'value': result.value,
                    'classification': result.classification,
                    'confidence': result.confidence,
                    'details': result.details,
                    'timestamp': time.time(),
                    # Forwarded straight from /supervisor/spectral_target so
                    # Supabase can attribute the reading to the right
                    # robot/task without relying on hardcoded UUIDs.
                    'robot_id': robot_id,
                    'task_id': task_id,
                }, separators=(',', ':'))
                self._result_pub.publish(String(data=result_payload))
                self.get_logger().info(
                    '[SPECTRO] Result published to /spectrometer/result')
            else:
                self.get_logger().error(
                    f'[SPECTRO] Pipeline returned NO result for "{task_type}" '
                    f'(status={status})')
        except Exception as e:
            self.get_logger().error(
                f'[SPECTRO] Exception in pipeline: {e}\n'
                f'{traceback.format_exc()}')
            self._publish_status('failure')
        finally:
            with self._lock:
                self._busy = False

    def destroy_node(self):
        try:
            self._pipeline.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpectrometerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('[SPECTRO] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
