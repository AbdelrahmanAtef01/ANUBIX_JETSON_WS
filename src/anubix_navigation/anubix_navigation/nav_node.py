#!/usr/bin/env python3
"""
ANUBIX Navigation Stack — Node (Raspberry Pi)
===============================================
Subscribes to: /supervisor/nav_goal     (geometry_msgs/PoseStamped) <- from Jetson master
               /supervisor/nav_vision   (std_msgs/Bool)             <- from Jetson master
               /supervisor/force_stop   (std_msgs/Bool)             <- from Jetson master
Publishes to:  /nav/status              (std_msgs/String)           -> to Jetson master

Status values: navigating | point_reached | blocked | failure

Vision flag:
  - True  → nav stops ~vision_standoff_m metres short of the goal so the
            on-board camera can take over the final approach.
  - False → nav drives all the way to the goal.

Preemption: A new nav_goal while navigating cancels the current goal and
starts the new one (matches Nav2 behavior).
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


class NavigationNode(Node):

    def __init__(self):
        super().__init__('anubix_navigation')

        self.declare_parameter('simulate', True)
        self.declare_parameter('nav_delay', 2.0)
        self.declare_parameter('vision_standoff_m', 1.0)
        self._simulate = self.get_parameter('simulate').value
        self._nav_delay = self.get_parameter('nav_delay').value
        self._vision_standoff_m = float(self.get_parameter('vision_standoff_m').value)
        self._force_stopped = False
        self._navigating = False
        self._preempted = False
        self._nav_lock = threading.Lock()
        self._vision_flag = False
        self._robot_id = ''
        self._task_id = ''
        self._ready = False

        self._sub_group = ReentrantCallbackGroup()

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PoseStamped, '/supervisor/nav_goal', self._on_nav_goal, cmd_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/nav_vision', self._on_nav_vision, cmd_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            String, '/supervisor/robot_id', self._on_robot_id, cmd_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            String, '/supervisor/task_id', self._on_task_id, cmd_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop, force_stop_qos,
            callback_group=self._sub_group)

        self._status_pub = self.create_publisher(String, '/nav/status', pub_qos)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  ANUBIX Navigation Node - Raspberry Pi')
        self.get_logger().info(
            f'  Mode: {"SIMULATE" if self._simulate else "HARDWARE"}')
        self.get_logger().info(f'  Nav delay: {self._nav_delay}s')
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            '[NAV] Subscribed to /supervisor/nav_goal (PoseStamped, TRANSIENT_LOCAL)')
        self.get_logger().info(
            '[NAV] Publishing on /nav/status (String, RELIABLE)')
        self.get_logger().info('[NAV] Ready and waiting for goals.')

        # Arm the node after a short delay so TRANSIENT_LOCAL late-join
        # messages from a previous session are discarded.
        self._arm_timer = self.create_timer(1.0, self._arm_node)

    def _arm_node(self):
        self._ready = True
        self._arm_timer.cancel()
        self.get_logger().info('[NAV] Node ARMED — accepting goals.')

    def _on_force_stop(self, msg: Bool):
        was = self._force_stopped
        self._force_stopped = bool(msg.data)
        if self._force_stopped:
            self._preempted = True
            self.get_logger().warning(
                '[NAV] *** FORCE STOP RECEIVED *** — aborting in-flight nav')
        elif was:
            self.get_logger().info(
                '[NAV] Force stop CLEARED — ready for new goals')

    def _on_nav_vision(self, msg: Bool):
        self._vision_flag = bool(msg.data)
        self.get_logger().info(
            f'[NAV] vision flag = {self._vision_flag} '
            f'(stop {self._vision_standoff_m:.2f} m short of goal '
            f'when True)')

    def _on_robot_id(self, msg: String):
        self._robot_id = msg.data.strip()
        self.get_logger().info(f'[NAV] robot_id = "{self._robot_id}"')

    def _on_task_id(self, msg: String):
        self._task_id = msg.data.strip()
        self.get_logger().info(f'[NAV] task_id = "{self._task_id}"')

    def _on_nav_goal(self, msg: PoseStamped):
        if not self._ready:
            self.get_logger().info(
                '[NAV] Ignoring TRANSIENT_LOCAL late-join message (node not armed yet)')
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        frame = msg.header.frame_id
        vision = self._vision_flag

        self.get_logger().info(
            f'[NAV] ========================================')
        self.get_logger().info(
            f'[NAV] Goal RECEIVED: ({x:.3f}, {y:.3f}) frame="{frame}" '
            f'vision={vision}')
        if self._robot_id or self._task_id:
            self.get_logger().info(
                f'[NAV] robot_id={self._robot_id!r} task_id={self._task_id!r}')
        if vision:
            self.get_logger().info(
                f'[NAV] vision=True → will stop ~{self._vision_standoff_m:.2f} m '
                f'short of ({x:.3f}, {y:.3f}); on-board camera handles the rest.')
        self.get_logger().info(
            f'[NAV] ========================================')

        if self._force_stopped:
            self.get_logger().warning(
                '[NAV] REJECTED: robot is force_stopped. Publishing "failure".')
            self._status_pub.publish(String(data='failure'))
            return

        with self._nav_lock:
            if self._navigating:
                self.get_logger().warning(
                    '[NAV] Preempting current navigation for new goal.')
                self._preempted = True
                # Let the old thread finish (it checks _preempted flag)
            self._navigating = True
            self._preempted = False

        self._status_pub.publish(String(data='navigating'))
        self.get_logger().info('[NAV] Published status: "navigating"')

        if self._simulate:
            threading.Thread(
                target=self._simulate_navigation,
                args=(x, y, vision),
                daemon=True).start()
        else:
            # TODO: Send goal to Nav2 action server.
            self.get_logger().warning(
                '[NAV] Hardware mode not yet implemented! '
                'Falling back to simulated delay.')
            threading.Thread(
                target=self._simulate_navigation,
                args=(x, y, vision),
                daemon=True).start()

    def _simulate_navigation(self, x: float, y: float, vision: bool):
        try:
            if vision:
                self.get_logger().info(
                    f'[NAV] Simulating navigation to '
                    f'({x:.3f}, {y:.3f}) but stopping '
                    f'~{self._vision_standoff_m:.2f} m short '
                    f'(vision=True) — waiting {self._nav_delay}s...')
            else:
                self.get_logger().info(
                    f'[NAV] Simulating navigation to ({x:.3f}, {y:.3f}) '
                    f'— waiting {self._nav_delay}s...')

            elapsed = 0.0
            step = 0.1
            while elapsed < self._nav_delay:
                if self._preempted or self._force_stopped:
                    self.get_logger().warning(
                        f'[NAV] Navigation PREEMPTED/STOPPED at {elapsed:.1f}s')
                    return
                time.sleep(step)
                elapsed += step

            if self._preempted or self._force_stopped:
                self.get_logger().warning(
                    f'[NAV] Navigation ABORTED after delay -> no status published (preempted)')
                return

            self._status_pub.publish(String(data='point_reached'))
            if vision:
                self.get_logger().info(
                    f'[NAV] Navigation COMPLETE (vision standoff) -> '
                    f'"point_reached" near ({x:.3f}, {y:.3f}) '
                    f'(stopped {self._vision_standoff_m:.2f} m short)')
            else:
                self.get_logger().info(
                    f'[NAV] Navigation COMPLETE -> "point_reached" '
                    f'at ({x:.3f}, {y:.3f})')
        except Exception as e:
            self.get_logger().error(
                f'[NAV] Exception during navigation: {e}\n'
                f'{traceback.format_exc()}')
            self._status_pub.publish(String(data='failure'))
        finally:
            with self._nav_lock:
                self._navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('[NAV] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
