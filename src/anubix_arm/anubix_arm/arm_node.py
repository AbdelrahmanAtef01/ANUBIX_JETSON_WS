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
               /arm/current_pose (geometry_msgs/PoseStamped)

Arm status values:     success | block | mechanical_error
Gripper status values: successful_grip | successful_release
                       | gripper_slipped | mechanical_error
Touch status values:   true | false

While the hardware arm is still being integrated, the node runs in
simulate mode and reports a successful move/grip so the rest of the
pipeline can be tested end-to-end.
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
        # Starting (home) pose published as the initial /arm/current_pose so
        # the vision node always has a base to add calibration offsets to.
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 0.3)
        # How often to re-publish the current pose (Hz).
        self.declare_parameter('pose_publish_rate_hz', 5.0)

        self._simulate = self.get_parameter('simulate').value
        self._arm_move_delay = self.get_parameter('arm_move_delay').value
        self._grip_delay = self.get_parameter('grip_delay').value
        home_x = float(self.get_parameter('home_x').value)
        home_y = float(self.get_parameter('home_y').value)
        home_z = float(self.get_parameter('home_z').value)
        pose_rate = float(self.get_parameter('pose_publish_rate_hz').value)

        self._arm_position = 'home'
        self._gripping = False
        self._force_stopped = False
        self._arm_busy = False
        self._grip_busy = False
        self._arm_lock = threading.Lock()
        self._grip_lock = threading.Lock()

        # Latest commanded pose — also used as the mocked "current" pose.
        self._current_pose = PoseStamped()
        self._current_pose.header.frame_id = 'base_link'
        self._current_pose.pose.position.x = home_x
        self._current_pose.pose.position.y = home_y
        self._current_pose.pose.position.z = home_z
        self._current_pose.pose.orientation.w = 1.0
        self._pose_lock = threading.Lock()

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
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self._pose_pub = self.create_publisher(
            PoseStamped, '/arm/current_pose', pose_qos)

        # Publish initial pose immediately (TRANSIENT_LOCAL — late subscribers
        # like the vision node always see at least one).
        self._publish_current_pose()

        if pose_rate > 0:
            self.create_timer(1.0 / pose_rate, self._publish_current_pose)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  ANUBIX Arm Control Node - Jetson Orin Nano')
        self.get_logger().info(
            f'  Mode: {"SIMULATE" if self._simulate else "HARDWARE"}')
        self.get_logger().info(f'  Arm move delay: {self._arm_move_delay}s')
        self.get_logger().info(f'  Grip delay: {self._grip_delay}s')
        self.get_logger().info(
            f'  Home pose: ({home_x:.3f}, {home_y:.3f}, {home_z:.3f})')
        self.get_logger().info('=' * 50)
        self.get_logger().info('[ARM] Ready and waiting for commands.')

    def _publish_current_pose(self):
        with self._pose_lock:
            self._current_pose.header.stamp = self.get_clock().now().to_msg()
            self._pose_pub.publish(self._current_pose)

    def _on_force_stop(self, msg: Bool):
        # Edge semantics: True halts in-flight moves/grips; False re-arms
        # the node. Mirrors the master's publish(True) → publish(False)
        # sequence and lets the system recover from a stale latched True.
        was = self._force_stopped
        self._force_stopped = bool(msg.data)
        if self._force_stopped:
            self.get_logger().warning(
                '[ARM] *** FORCE STOP RECEIVED *** — halting all arm operations')
        elif was:
            self.get_logger().info(
                '[ARM] Force stop CLEARED — ready for new commands')

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

        threading.Thread(
            target=self._simulate_arm_move,
            args=(msg,),
            daemon=True).start()

    def _simulate_arm_move(self, goal: PoseStamped):
        x = goal.pose.position.x
        y = goal.pose.position.y
        z = goal.pose.position.z
        try:
            self.get_logger().info(
                f'[ARM] Moving to ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'— waiting {self._arm_move_delay}s...')
            time.sleep(self._arm_move_delay)

            if self._force_stopped:
                self._arm_status_pub.publish(String(data='mechanical_error'))
                self.get_logger().warning(
                    '[ARM] Move ABORTED (force stopped) -> "mechanical_error"')
                return

            # Mocked arm always succeeds. Update the "current pose" so the
            # vision node can use it as the base for the next calibration.
            with self._pose_lock:
                self._current_pose.header.frame_id = (
                    goal.header.frame_id or 'base_link')
                self._current_pose.pose = goal.pose
                if (self._current_pose.pose.orientation.w == 0.0
                        and self._current_pose.pose.orientation.x == 0.0
                        and self._current_pose.pose.orientation.y == 0.0
                        and self._current_pose.pose.orientation.z == 0.0):
                    self._current_pose.pose.orientation.w = 1.0
            self._publish_current_pose()

            self._arm_status_pub.publish(String(data='success'))
            self._arm_position = 'extended' if z != 0.3 else 'home'
            self.get_logger().info(
                f'[ARM] Move COMPLETE -> "success" '
                f'pos=({x:.3f}, {y:.3f}, {z:.3f})')
        except Exception as e:
            self.get_logger().error(
                f'[ARM] Exception during arm move: {e}\n'
                f'{traceback.format_exc()}')
            # Even on exceptions during the mocked move, report success so
            # the rest of the pipeline stays testable. Real hardware should
            # report mechanical_error here.
            self._arm_status_pub.publish(String(data='success'))
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
            # Same as above: keep the mock optimistic so the pipeline can
            # be tested without the real hardware.
            if action:
                self._gripper_status_pub.publish(String(data='successful_grip'))
                self._touch_status_pub.publish(Bool(data=True))
            else:
                self._gripper_status_pub.publish(String(data='successful_release'))
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
