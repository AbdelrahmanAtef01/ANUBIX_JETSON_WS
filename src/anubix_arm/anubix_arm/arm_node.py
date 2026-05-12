#!/usr/bin/env python3
"""
ANUBIX Arm Control Stack - Node (Jetson Orin Nano)
====================================================
Subscribes to: /supervisor/arm_nav_goal (geometry_msgs/PoseStamped)
               /supervisor/grip (std_msgs/Bool)
               /supervisor/force_stop (std_msgs/Bool)
Publishes to:  /arm/arm_status (std_msgs/String)
               /arm/gripper_status (std_msgs/String)
               /arm/touch_status (std_msgs/Bool)

Arm status values:     success | block | mechanical_error
Gripper status values: successful_grip | successful_release
                       | gripper_slipped | mechanical_error
Touch status values:   true | false
"""

import time
import threading
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped


class ArmNode(Node):

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
        self._force_stopped = False
        self._arm_busy = False
        self._grip_busy = False
        self._arm_lock = threading.Lock()
        self._grip_lock = threading.Lock()

        self._sub_group = ReentrantCallbackGroup()

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
            PoseStamped, '/supervisor/arm_nav_goal', self._on_arm_goal, cmd_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/grip', self._on_grip, cmd_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop, cmd_qos,
            callback_group=self._sub_group)

        # Publishers
        self._arm_status_pub = self.create_publisher(
            String, '/arm/arm_status', pub_qos)
        self._gripper_status_pub = self.create_publisher(
            String, '/arm/gripper_status', pub_qos)
        self._touch_status_pub = self.create_publisher(
            Bool, '/arm/touch_status', pub_qos)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  ANUBIX Arm Control Node - Jetson Orin Nano')
        self.get_logger().info(
            f'  Mode: {"SIMULATE" if self._simulate else "HARDWARE"}')
        self.get_logger().info(f'  Arm move delay: {self._arm_move_delay}s')
        self.get_logger().info(f'  Grip delay: {self._grip_delay}s')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            '[ARM] Subscribed to /supervisor/arm_nav_goal (PoseStamped)')
        self.get_logger().info(
            '[ARM] Subscribed to /supervisor/grip (Bool)')
        self.get_logger().info(
            '[ARM] Publishing on /arm/arm_status, /arm/gripper_status, /arm/touch_status')
        self.get_logger().info('[ARM] Ready and waiting for commands.')

    def _on_force_stop(self, msg: Bool):
        if msg.data:
            self._force_stopped = True
            self.get_logger().warning(
                '[ARM] *** FORCE STOP RECEIVED *** — halting all arm operations')

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

        if self._simulate:
            threading.Thread(
                target=self._simulate_arm_move,
                args=(x, y, z, frame),
                daemon=True).start()
        else:
            # TODO: Replace with MoveIt2 action client
            self.get_logger().warning(
                '[ARM] Hardware mode not yet implemented! Using simulated delay.')
            threading.Thread(
                target=self._simulate_arm_move,
                args=(x, y, z, frame),
                daemon=True).start()

    def _simulate_arm_move(self, x: float, y: float, z: float, frame: str):
        try:
            self.get_logger().info(
                f'[ARM] Moving to ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'— waiting {self._arm_move_delay}s...')
            time.sleep(self._arm_move_delay)

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Move ABORTED (force stopped) -> "mechanical_error"')
            else:
                self._arm_status_pub.publish(String(data='success'))
                self._arm_position = 'extended' if z != 0.3 else 'home'
                self.get_logger().info(
                    f'[ARM] Move COMPLETE -> "success" '
                    f'pos=({x:.3f}, {y:.3f}, {z:.3f})')
        except Exception as e:
            self.get_logger().error(
                f'[ARM] Exception during arm move: {e}\n'
                f'{traceback.format_exc()}')
            self._arm_status_pub.publish(String(data='mechanical_error'))
        finally:
            with self._arm_lock:
                self._arm_busy = False

    def _on_grip(self, msg: Bool):
        action = msg.data
        action_str = "CLOSE (grip)" if action else "OPEN (release)"

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

        if self._simulate:
            threading.Thread(
                target=self._simulate_grip,
                args=(action,),
                daemon=True).start()
        else:
            # TODO: Replace with gripper action server
            self.get_logger().warning(
                '[ARM] Hardware grip not yet implemented! Using simulated delay.')
            threading.Thread(
                target=self._simulate_grip,
                args=(action,),
                daemon=True).start()

    def _simulate_grip(self, action: bool):
        try:
            action_str = "close" if action else "open"
            self.get_logger().info(
                f'[ARM] Gripper {action_str} — waiting {self._grip_delay}s...')
            time.sleep(self._grip_delay)

            if self._force_stopped:
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Grip ABORTED (force stopped) -> "mechanical_error"')
                return

            if action:
                self._gripping = True
                self._gripper_status_pub.publish(String(data='successful_grip'))
                self.get_logger().info(
                    '[ARM] Gripper CLOSED -> "successful_grip"')
                # Publish touch status
                self._touch_status_pub.publish(Bool(data=True))
                self.get_logger().info('[ARM] Touch sensor -> true')
            else:
                self._gripping = False
                self._gripper_status_pub.publish(String(data='successful_release'))
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
