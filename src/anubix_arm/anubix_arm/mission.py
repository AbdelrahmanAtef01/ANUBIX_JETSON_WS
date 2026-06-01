#!/usr/bin/env python3
"""
mission.py  —  test runner for anubix_arm
Sends a sequence of PoseStamped goals and waits for each to finish.

Usage:
    ros2 run anubix_arm mission                    # runs all POINTS in order
    ros2 run anubix_arm mission -- 3 6 7           # runs only points 3, 6, 7

Or standalone:
    python3 mission.py
    python3 mission.py 3 6 7
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

_QOS_GOAL = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_QOS_STATUS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Covers the full reachable envelope.
# Points 3, 6, 7 are the historically problematic J6-J2/J3 collision zone.
POINTS = [
    ( 0.25,  0.00,  0.40,  "1  front-centre high          — safe baseline"),
    ( 0.20,  0.15,  0.28,  "2  front-right mid"),
    ( 0.10,  0.00,  0.15,  "3  close + low                — wrist near body"),
    ( 0.00,  0.25,  0.30,  "4  pure side reach"),
    (-0.20,  0.00,  0.35,  "5  rear-centre"),
    ( 0.30,  0.00,  0.20,  "6  extended + low             — elbow stress"),
    ( 0.15, -0.15,  0.18,  "7  front-left low             — old collision zone"),
]


class MissionRunner(Node):

    def __init__(self):
        super().__init__('mission_runner')

        self._pub = self.create_publisher(
            PoseStamped, '/supervisor/arm_nav_goal', _QOS_GOAL
        )
        self._sub = self.create_subscription(
            String, '/arm/arm_status', self._on_status, _QOS_STATUS
        )
        self._event       = threading.Event()
        self._last_status = None

    def _on_status(self, msg: String):
        self._last_status = msg.data
        if msg.data in ('success', 'preflight_failed', 'mechanical_error'):
            self._event.set()

    def send(self, x: float, y: float, z: float, label: str):
        self.get_logger().info(f'-- Sending point {label}  [{x}, {y}, {z}] m')
        msg = PoseStamped()
        msg.header.frame_id     = 'base_link'
        msg.pose.position.x     = x
        msg.pose.position.y     = y
        msg.pose.position.z     = z
        msg.pose.orientation.w  = 1.0
        self._event.clear()
        self._last_status = None
        self._pub.publish(msg)

    def wait(self, timeout: float = 45.0) -> bool:
        done = self._event.wait(timeout=timeout)
        if not done:
            self.get_logger().error('TIMEOUT — no status received within limit')
            return False
        status = self._last_status
        ok = status == 'success'
        level = self.get_logger().info if ok else self.get_logger().error
        level(f'   -> status: {status}')
        return ok


def main():
    rclpy.init()
    node = MissionRunner()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    time.sleep(0.8)

    if len(sys.argv) > 1:
        indices = [int(a) - 1 for a in sys.argv[1:]]
        points  = [(POINTS[i], i + 1) for i in indices if 0 <= i < len(POINTS)]
    else:
        points = [(p, i + 1) for i, p in enumerate(POINTS)]

    passed = failed = 0
    for (x, y, z, label), num in points:
        node.send(x, y, z, label)
        ok = node.wait(timeout=45)
        if ok:
            passed += 1
        else:
            failed += 1
            node.get_logger().error(f'Mission aborted at point {num}')
            break
        time.sleep(0.4)

    node.get_logger().info(
        f'Mission done — passed: {passed}  failed: {failed}'
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
