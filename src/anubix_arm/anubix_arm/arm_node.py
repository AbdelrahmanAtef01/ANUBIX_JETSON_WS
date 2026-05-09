#!/usr/bin/env python3
"""
ANUBIX Arm Control Stack - Placeholder Node
=============================================
Subscribes to: /supervisor/arm_nav_goal (geometry_msgs/PoseStamped)
               /supervisor/grip (std_msgs/Bool)
Publishes to:  /arm/arm_status (std_msgs/String)
               /arm/gripper_status (std_msgs/String)
               /arm/touch_status (std_msgs/Bool)

Arm status values:     success | block | mechanical_error
Gripper status values: successful_grip | successful_release
                       | gripper_slipped | mechanical_error
Touch status values:   true | false

TODO: Integrate actual arm control:
      - MoveIt2 for motion planning
      - Joint trajectory controller interface
      - Gripper action server (parallel jaw / servo gripper)
      - Touch/force sensor driver
      - Collision avoidance with the crop
      - Inverse kinematics for the specific arm model
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped


class ArmNode(Node):
    """
    Placeholder arm control node.
    Receives arm navigation goals and grip commands.
    Publishes arm status, gripper status, and touch sensor readings.
    """

    def __init__(self):
        super().__init__('anubix_arm')

        # Parameters
        self.declare_parameter('simulate', True)
        self.declare_parameter('arm_move_delay', 2.0)
        self.declare_parameter('grip_delay', 1.0)
        self._simulate = self.get_parameter('simulate').value
        self._arm_move_delay = self.get_parameter('arm_move_delay').value
        self._grip_delay = self.get_parameter('grip_delay').value

        self._arm_position = 'home'
        self._gripping = False

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

        # Subscribers
        self.create_subscription(
            PoseStamped, '/supervisor/arm_nav_goal', self._on_arm_goal, cmd_qos)
        self.create_subscription(
            Bool, '/supervisor/grip', self._on_grip, cmd_qos)

        # Publishers
        self._arm_status_pub = self.create_publisher(
            String, '/arm/arm_status', pub_qos)
        self._gripper_status_pub = self.create_publisher(
            String, '/arm/gripper_status', pub_qos)
        self._touch_status_pub = self.create_publisher(
            Bool, '/arm/touch_status', pub_qos)

        self.get_logger().info('[ARM] Arm control node ready (placeholder mode)')

    def _on_arm_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info(f'[ARM] Goal received: ({x:.3f}, {y:.3f}, {z:.3f})')

        if self._simulate:
            # TODO: Replace with MoveIt2 action client
            self.create_timer(
                self._arm_move_delay,
                lambda: self._complete_arm_move(x, y, z),
            )
        else:
            # TODO: Send goal to MoveIt2 / joint trajectory controller
            pass

    def _complete_arm_move(self, x: float, y: float, z: float):
        # TODO: Check actual arm motion result
        self._arm_status_pub.publish(String(data='success'))
        self.get_logger().info(f'[ARM] Move complete: ({x:.3f}, {y:.3f}, {z:.3f})')

    def _on_grip(self, msg: Bool):
        action = msg.data  # True = close/grip, False = open/release
        self.get_logger().info(f'[ARM] Grip command: {"close" if action else "open"}')

        if self._simulate:
            # TODO: Replace with gripper action server
            self.create_timer(
                self._grip_delay,
                lambda: self._complete_grip(action),
            )
        else:
            # TODO: Send to gripper controller
            pass

    def _complete_grip(self, action: bool):
        if action:
            # Close gripper
            self._gripping = True
            self._gripper_status_pub.publish(String(data='successful_grip'))
            # TODO: Read actual touch/force sensor
            self._touch_status_pub.publish(Bool(data=True))
            self.get_logger().info('[ARM] Grip closed - touch confirmed')
        else:
            # Open gripper
            self._gripping = False
            self._gripper_status_pub.publish(String(data='successful_release'))
            self.get_logger().info('[ARM] Grip released')


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
