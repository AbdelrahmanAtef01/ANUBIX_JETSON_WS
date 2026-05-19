#!/usr/bin/env python3
"""
mission.py — Arm waypoint test harness
========================================
Sends a sequence of 8 predefined waypoints to /supervisor/arm_nav_goal
and waits for /arm/arm_status confirmation after each one.

Usage:
    ros2 run anubix_arm mission
"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


POINTS = [
    # (x,    y,     z,    label)         — all in metres, base_link frame
    ( 0.20,  0.00,  0.40, 'point 1 — front centre high'),
    ( 0.20,  0.10,  0.30, 'point 2 — front right mid'),
    ( 0.15,  0.15,  0.25, 'point 3 — front right low'),
    ( 0.00,  0.20,  0.30, 'point 4 — side'),
    (-0.15,  0.15,  0.35, 'point 5 — back right'),
    (-0.20,  0.00,  0.40, 'point 6 — back centre'),
    ( 0.10,  0.00,  0.20, 'point 7 — close low'),
    ( 0.20,  0.00,  0.40, 'point 8 — return front'),
]

TERMINAL_STATUSES = ('success', 'preflight_failed', 'mechanical_error')


class MissionRunner(Node):

    def __init__(self):
        super().__init__('mission_runner')
        self._pub = self.create_publisher(
            PoseStamped, '/supervisor/arm_nav_goal', 10)
        self._sub = self.create_subscription(
            String, '/arm/arm_status', self._on_status, 10)
        self._event = threading.Event()
        self._last_status = None

    def _on_status(self, msg):
        self._last_status = msg.data
        if msg.data in TERMINAL_STATUSES:
            self._event.set()

    def send(self, x, y, z, label):
        self.get_logger().info(
            f'--- Sending: {label}  [{x}, {y}, {z}] m')
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._event.clear()
        self._pub.publish(msg)

    def wait(self, timeout=60.0):
        done = self._event.wait(timeout=timeout)
        if not done:
            self.get_logger().error('TIMEOUT — no status received')
            return False
        self.get_logger().info(f'    status: {self._last_status}')
        return self._last_status == 'success'


def main():
    rclpy.init()
    node = MissionRunner()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    time.sleep(1)

    for i, (x, y, z, label) in enumerate(POINTS):
        node.send(x, y, z, label)
        ok = node.wait(timeout=60)
        if not ok:
            node.get_logger().error(
                f'Mission aborted at point {i + 1}: {label}')
            break
        time.sleep(0.5)

    node.get_logger().info('Mission complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
