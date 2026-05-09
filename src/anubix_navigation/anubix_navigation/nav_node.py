#!/usr/bin/env python3
"""
ANUBIX Navigation Stack - Placeholder Node
============================================
Subscribes to: /supervisor/nav_goal (geometry_msgs/PoseStamped)
Publishes to:  /nav/status (std_msgs/String)

Status values: point_reached | navigating | blocked | failure

TODO: Integrate with Nav2 or custom path planner for the Jetson Orin Nano.
      - Replace placeholder logic with actual navigation action client
      - Add obstacle avoidance via costmap
      - Add GPS/RTK waypoint conversion for outdoor operation
      - Add odometry fusion (wheel encoders + IMU + GPS)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class NavigationNode(Node):
    """
    Placeholder navigation node.
    Receives PoseStamped goals from the master and publishes navigation status.
    """

    def __init__(self):
        super().__init__('anubix_navigation')

        # Parameters
        self.declare_parameter('simulate', True)
        self.declare_parameter('nav_delay', 2.0)
        self._simulate = self.get_parameter('simulate').value
        self._nav_delay = self.get_parameter('nav_delay').value

        # QoS
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

        # Subscriber
        self.create_subscription(
            PoseStamped, '/supervisor/nav_goal', self._on_nav_goal, cmd_qos)

        # Publisher
        self._status_pub = self.create_publisher(String, '/nav/status', pub_qos)

        self.get_logger().info('[NAV] Navigation node ready (placeholder mode)')

    def _on_nav_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'[NAV] Goal received: ({x:.2f}, {y:.2f})')

        # Publish navigating status
        self._status_pub.publish(String(data='navigating'))

        if self._simulate:
            # TODO: Replace with actual Nav2 action client
            # For now, simulate navigation with a timer callback
            self.create_timer(
                self._nav_delay,
                lambda: self._complete_navigation(x, y),
            )
        else:
            # TODO: Send goal to Nav2 action server
            # nav2_msgs/action/NavigateToPose
            pass

    def _complete_navigation(self, x: float, y: float):
        # TODO: Check actual navigation result from Nav2
        self._status_pub.publish(String(data='point_reached'))
        self.get_logger().info(f'[NAV] Reached ({x:.2f}, {y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
