#!/usr/bin/env python3
"""
ANUBIX Spectrometer Stack - ROS 2 Node
========================================
Subscribes to: /supervisor/spectral_target (std_msgs/String)
               /supervisor/force_stop (std_msgs/Bool)
Publishes to:  /spectrometer/status (std_msgs/String)

Status values: reading | applying_ML | uploading | success | failure

Integrates the SpectrometerPipeline driver with ROS 2 topics.
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

from anubix_spectrometer.spectrometer_driver import (
    SpectrometerPipeline,
    SpectrometerDevice,
)


class SpectrometerNode(Node):
    """
    ROS 2 node wrapping the ANUBIX spectrometer pipeline.
    Receives spectral analysis requests, runs the full pipeline
    (acquire -> calibrate -> analyze -> upload), and publishes status
    at each stage.
    """

    def __init__(self):
        super().__init__('anubix_spectrometer')

        # Parameters
        self.declare_parameter('simulate', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('num_channels', 257)
        self.declare_parameter('integration_time_ms', 100)
        self.declare_parameter('bg_file', '')

        self._simulate = self.get_parameter('simulate').value
        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._num_channels = self.get_parameter('num_channels').value
        self._integration_time_ms = self.get_parameter('integration_time_ms').value
        bg_file_param = self.get_parameter('bg_file').value

        # Locate bg.csv
        if bg_file_param:
            self._bg_path = bg_file_param
        else:
            try:
                pkg_share = get_package_share_directory('anubix_spectrometer')
                self._bg_path = os.path.join(pkg_share, 'config', 'bg.csv')
            except Exception:
                self._bg_path = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'bg.csv')

        if not os.path.exists(self._bg_path):
            self.get_logger().fatal(f'Background file not found: {self._bg_path}')
            raise FileNotFoundError(self._bg_path)

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
            String, '/supervisor/spectral_target', self._on_spectral_target, cmd_qos)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop, cmd_qos)

        # Publisher
        self._status_pub = self.create_publisher(String, '/spectrometer/status', pub_qos)

        # Initialize pipeline
        device = None
        if not self._simulate:
            device = SpectrometerDevice(
                port=self._serial_port,
                baud_rate=self._baud_rate,
                num_channels=self._num_channels,
                integration_time_ms=self._integration_time_ms,
            )

        self._pipeline = SpectrometerPipeline(
            bg_path=self._bg_path,
            device=device,
            simulate=self._simulate,
        )
        self._pipeline.set_status_callback(self._publish_status)
        self._pipeline.connect()

        self._busy = False
        self._force_stopped = False
        self._lock = threading.Lock()

        mode = 'simulated' if self._simulate else self._serial_port
        self.get_logger().info(
            f'[SPECTRO] Node ready | mode={mode} | '
            f'channels={self._num_channels} | bg={self._bg_path}')

    def _publish_status(self, status: str):
        """Callback from pipeline - publish status to ROS topic."""
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f'[SPECTRO] Status: {status}')

    def _on_force_stop(self, msg: Bool):
        if msg.data:
            self._force_stopped = True
            self.get_logger().warning('[SPECTRO] Force stop received')

    def _on_spectral_target(self, msg: String):
        task_type = msg.data.strip().lower()
        self.get_logger().info(f'[SPECTRO] Target received: {task_type}')

        if self._force_stopped:
            self.get_logger().warning('[SPECTRO] Ignoring - force stopped')
            self._publish_status('failure')
            return

        with self._lock:
            if self._busy:
                self.get_logger().warning('[SPECTRO] Already processing - ignoring')
                return
            self._busy = True

        # Run pipeline in a thread to avoid blocking the executor
        thread = threading.Thread(
            target=self._run_pipeline, args=(task_type,), daemon=True)
        thread.start()

    def _run_pipeline(self, task_type: str):
        """Execute the full pipeline in a background thread."""
        try:
            status, result = self._pipeline.run(task_type)

            if result:
                self.get_logger().info(
                    f'[SPECTRO] Result: {result.classification} '
                    f'(confidence={result.confidence:.2%}, '
                    f'details={result.details})')
            else:
                self.get_logger().error(f'[SPECTRO] Pipeline failed for {task_type}')
        except Exception as e:
            self.get_logger().error(f'[SPECTRO] Exception: {e}')
            self._publish_status('failure')
        finally:
            with self._lock:
                self._busy = False

    def destroy_node(self):
        """Clean shutdown."""
        try:
            self._pipeline.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpectrometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
