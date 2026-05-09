#!/usr/bin/env python3
"""
ANUBIX ROS 2 Master Node
=========================
Bridges the ANUBIX OmniLink agent to four ROS 2 subsystems on the Jetson Orin Nano:

  1. Navigation Stack       (/supervisor/nav_goal           -> /nav/status)
  2. Perception Stack       (/supervisor/perception_goal,
                             /supervisor/target_camera      -> /perception/status)
  3. Arm Control Stack      (/supervisor/arm_nav_goal,
                             /supervisor/grip               -> /arm/arm_status,
                                                              /arm/gripper_status,
                                                              /arm/touch_status)
  4. Spectrometer Stack     (/supervisor/spectral_target    -> /spectrometer/status)

Run:
    source /opt/ros/humble/setup.bash
    source ~/anubix_ws/install/setup.bash
    export OMNI_KEY=olink_YOUR_KEY_HERE
    ros2 run anubix_master master_node
"""

import os
import re
import sys
import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose

try:
    from omnilink.client import OmniLinkClient, OmniLinkAPIError
except ImportError:
    print("ERROR: omnilink not installed. Run: pip install omnilink")
    sys.exit(1)

from anubix_master.command_parser import parse_commands, CMD_PRIORITY


class AnubixMasterNode(Node):
    """
    ROS 2 node that publishes supervisor commands and subscribes to stack feedback.
    Integrates with OmniLink to receive AI agent commands and relay hardware feedback.
    """

    def __init__(self):
        super().__init__('anubix_master')

        # Parameters
        self.declare_parameter('omni_key', '')
        self.declare_parameter('poll_interval', 3.0)
        self.declare_parameter('feedback_timeout', 30.0)
        self.declare_parameter('arm_home_x', 0.0)
        self.declare_parameter('arm_home_y', 0.0)
        self.declare_parameter('arm_home_z', 0.3)
        self.declare_parameter('robot_id', '34a957fd-d45c-4dbf-8e02-be8e1b5e349a')
        self.declare_parameter('task_id',  '40e4060b-5bc8-4044-9d71-046fee27a757')

        omni_key = self.get_parameter('omni_key').value or os.environ.get('OMNI_KEY', '')
        if not omni_key:
            self.get_logger().fatal('OMNI_KEY not set. Pass as param or env var.')
            sys.exit(1)

        self.poll_interval = self.get_parameter('poll_interval').value
        self.feedback_timeout = self.get_parameter('feedback_timeout').value
        self.arm_home_pose = (
            self.get_parameter('arm_home_x').value,
            self.get_parameter('arm_home_y').value,
            self.get_parameter('arm_home_z').value,
        )
        self._robot_id = self.get_parameter('robot_id').value
        self._task_id  = self.get_parameter('task_id').value

        # OmniLink client
        self.client = OmniLinkClient(omni_key=omni_key, timeout=120)
        self.AGENT_NAME = "ANUBIX"
        self.ENGINE = "g1-engine"

        # QoS profiles
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers (supervisor command channels)
        self.pub_nav_goal = self.create_publisher(PoseStamped, '/supervisor/nav_goal', cmd_qos)
        self.pub_perception = self.create_publisher(String, '/supervisor/perception_goal', cmd_qos)
        self.pub_target_camera = self.create_publisher(String, '/supervisor/target_camera', cmd_qos)
        self.pub_arm_nav_goal = self.create_publisher(PoseStamped, '/supervisor/arm_nav_goal', cmd_qos)
        self.pub_grip = self.create_publisher(Bool, '/supervisor/grip', cmd_qos)
        self.pub_spectral = self.create_publisher(String, '/supervisor/spectral_target', cmd_qos)
        self.pub_force_stop = self.create_publisher(Bool, '/supervisor/force_stop', cmd_qos)

        # Feedback synchronization
        self._ev_nav = threading.Event()
        self._ev_perception = threading.Event()
        self._ev_arm = threading.Event()
        self._ev_gripper = threading.Event()
        self._ev_touch = threading.Event()
        self._ev_spectro = threading.Event()

        self._fb_nav: Optional[str] = None
        self._fb_perception: Optional[str] = None
        self._fb_arm: Optional[str] = None
        self._fb_gripper: Optional[str] = None
        self._fb_touch: Optional[bool] = None
        self._fb_spectro: Optional[str] = None

        self._latest_target_pose: Optional[Pose] = None
        self._target_pose_lock = threading.Lock()
        self._post_release_retract: bool = False

        # Subscribers (feedback from stacks)
        self.create_subscription(String, '/nav/status', self._cb_nav, sub_qos)
        self.create_subscription(String, '/perception/status', self._cb_perception, sub_qos)
        self.create_subscription(String, '/arm/arm_status', self._cb_arm, sub_qos)
        self.create_subscription(String, '/arm/gripper_status', self._cb_gripper, sub_qos)
        self.create_subscription(Bool, '/arm/touch_status', self._cb_touch, sub_qos)
        self.create_subscription(String, '/spectrometer/status', self._cb_spectro, sub_qos)
        self.create_subscription(Pose, '/perception/target_pose', self._cb_target_pose, sub_qos)

        # Internal state
        self._position = (0.0, 0.0)
        self._home = (0.0, 0.0)
        self._current_camera = 1
        self._current_task = ''
        self._arm_position = 'home'
        self._gripping = False
        self._mission_active = False
        self._force_stopped = False

        # Memory polling state
        self._mem_len = 0
        self._last_seen_fingerprint: Optional[str] = None
        self._running = True

        # OmniLink poll timer
        self._poll_timer = self.create_timer(self.poll_interval, self._poll_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ANUBIX ROS 2 Master Node - Jetson Orin Nano')
        self.get_logger().info('  4-stack architecture: NAV | PERCEPTION | ARM | SPECTRO')
        self.get_logger().info('=' * 60)

        # Stabilize memory cursor before starting poll
        self._sync_memory_stable()
        self.get_logger().info(
            f'[READY] Listening on agent "{self.AGENT_NAME}" '
            f'(memory at {self._mem_len} msgs)')

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _cb_nav(self, msg):
        self._fb_nav = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /nav/status = {self._fb_nav}')
        self._ev_nav.set()

    def _cb_perception(self, msg):
        self._fb_perception = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /perception/status = {self._fb_perception}')
        self._ev_perception.set()

    def _cb_arm(self, msg):
        self._fb_arm = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /arm/arm_status = {self._fb_arm}')
        self._ev_arm.set()

    def _cb_gripper(self, msg):
        self._fb_gripper = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /arm/gripper_status = {self._fb_gripper}')
        self._ev_gripper.set()

    def _cb_touch(self, msg):
        self._fb_touch = bool(msg.data)
        self.get_logger().info(f'[RX] /arm/touch_status = {self._fb_touch}')
        self._ev_touch.set()

    def _cb_spectro(self, msg):
        self._fb_spectro = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /spectrometer/status = {self._fb_spectro}')
        if self._fb_spectro in ('success', 'failure'):
            self._ev_spectro.set()

    def _cb_target_pose(self, msg):
        with self._target_pose_lock:
            self._latest_target_pose = msg
        self.get_logger().info(
            f'[RX] /perception/target_pose = '
            f'({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})')

    # ── Command execution ─────────────────────────────────────────────────────

    def execute_command(self, cmd_type: str, **kwargs) -> Optional[str]:
        if self._force_stopped and cmd_type != 'force_stop':
            self.get_logger().warning('Ignoring command - robot is force_stopped')
            return None

        handler = getattr(self, f'_do_{cmd_type}', None)
        if handler is None:
            self.get_logger().error(f'No handler for command: {cmd_type}')
            return None
        return handler(**kwargs)

    def _do_force_stop(self) -> str:
        self.pub_force_stop.publish(Bool(data=True))
        self._force_stopped = True
        self._mission_active = False
        self.get_logger().warning('*** FORCE STOP PUBLISHED ***')
        return '/system/status: force_stopped'

    def _do_nav_goal(self, x: float, y: float) -> str:
        ps = self._make_pose_stamped(x, y, 0.0)
        self._ev_nav.clear()
        self._fb_nav = None
        self.pub_nav_goal.publish(ps)
        self.get_logger().info(f'[TX] /supervisor/nav_goal ({x:.2f}, {y:.2f})')
        if not self._ev_nav.wait(self.feedback_timeout):
            return '/nav/status: failure'
        self._position = (x, y)
        self._mission_active = True
        return f'/nav/status: {self._fb_nav}'

    def _do_nav_goal_home(self) -> str:
        return self._do_nav_goal(self._home[0], self._home[1])

    def _do_target_camera(self, camera_number: int) -> None:
        self.pub_target_camera.publish(String(data=str(camera_number)))
        self._current_camera = camera_number
        self.get_logger().info(f'[TX] /supervisor/target_camera {camera_number}')
        return None

    def _do_perception_goal(self, task_type: str) -> str:
        self._ev_perception.clear()
        self._fb_perception = None
        self.pub_perception.publish(String(data=task_type))
        self._current_task = task_type
        self.get_logger().info(f'[TX] /supervisor/perception_goal {task_type}')
        if not self._ev_perception.wait(self.feedback_timeout):
            return '/perception/status: not_found'
        return f'/perception/status: {self._fb_perception}'

    def _do_arm_nav_goal(self, signal: str) -> str:
        target_is_home = (signal == 'home') or self._post_release_retract

        if target_is_home:
            ps = self._make_pose_stamped(
                *self.arm_home_pose, frame_id='base_link')
            if signal == 'move' and self._post_release_retract:
                self.get_logger().info(
                    'Post-release retract: overriding move with home pose')
        elif signal == 'move':
            with self._target_pose_lock:
                tgt = self._latest_target_pose
            if tgt is None:
                self.get_logger().error(
                    'arm_nav_goal_move REFUSED: no target_pose received')
                return '/arm/arm_status: mechanical_error'
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'base_link'
            ps.pose = tgt
        else:
            self.get_logger().error(f'Unknown arm signal: {signal}')
            return '/arm/arm_status: mechanical_error'

        self._ev_arm.clear()
        self._fb_arm = None
        self.pub_arm_nav_goal.publish(ps)
        self.get_logger().info(
            f'[TX] /supervisor/arm_nav_goal (signal={signal}, '
            f'dest={"home" if target_is_home else "target"})')
        if not self._ev_arm.wait(self.feedback_timeout):
            return '/arm/arm_status: mechanical_error'

        if target_is_home and self._fb_arm == 'success':
            self._post_release_retract = False
            self._arm_position = 'home'
        elif self._fb_arm == 'success':
            self._arm_position = 'extended'
        return f'/arm/arm_status: {self._fb_arm}'

    def _do_grip(self, action: bool) -> str:
        self._ev_gripper.clear()
        self._ev_touch.clear()
        self._fb_gripper = None
        self._fb_touch = None
        self.pub_grip.publish(Bool(data=action))
        self.get_logger().info(f'[TX] /supervisor/grip {action}')

        if not self._ev_gripper.wait(self.feedback_timeout):
            return '/arm/gripper_status: mechanical_error'
        gripper_line = f'/arm/gripper_status: {self._fb_gripper}'

        if not action:
            self._gripping = False
            if self._fb_gripper == 'successful_release':
                self._post_release_retract = True
            return gripper_line

        if not self._ev_touch.wait(self.feedback_timeout):
            touch_line = '/arm/touch_status: false'
        else:
            touch_line = f'/arm/touch_status: {"true" if self._fb_touch else "false"}'
        self._gripping = (
            bool(self._fb_touch) and self._fb_gripper == 'successful_grip')
        return f'{gripper_line}\n{touch_line}'

    def _do_spectral_target(self, task_type: str) -> str:
        self._ev_spectro.clear()
        self._fb_spectro = None
        self.pub_spectral.publish(String(data=task_type))
        self.get_logger().info(f'[TX] /supervisor/spectral_target {task_type}')
        if not self._ev_spectro.wait(self.feedback_timeout):
            return '/spectrometer/status: failure'
        feedback = f'/spectrometer/status: {self._fb_spectro}'
        if self._fb_spectro == 'success':
            feedback += f'\nrobot_id: {self._robot_id}\ntask_id: {self._task_id}'
        return feedback

    # ── OmniLink polling ──────────────────────────────────────────────────────

    def _poll_callback(self):
        if not self._running:
            return
        try:
            self._poll()
        except OmniLinkAPIError as e:
            self.get_logger().error(f'[API] {e.status_code}: {e.body}')
        except Exception as e:
            self.get_logger().error(f'[ERROR] {e}')

    @staticmethod
    def _fingerprint(msg: dict) -> str:
        role = msg.get('role', '')
        text = ''.join(p.get('text', '') for p in msg.get('parts', []))
        return f'{role}::{text}'

    def _sync_memory_stable(self, max_attempts: int = 8, delay: float = 0.5):
        prev_len = -1
        prev_fp = None
        for _ in range(max_attempts):
            memory = self.client.get_memory(self.AGENT_NAME) or []
            cur_len = len(memory)
            cur_fp = self._fingerprint(memory[-1]) if memory else None
            if cur_len == prev_len and cur_fp == prev_fp:
                self._mem_len = cur_len
                self._last_seen_fingerprint = cur_fp
                return
            prev_len, prev_fp = cur_len, cur_fp
            time.sleep(delay)
        self._mem_len = prev_len if prev_len >= 0 else 0
        self._last_seen_fingerprint = prev_fp

    def _sync_memory(self):
        memory = self.client.get_memory(self.AGENT_NAME) or []
        self._mem_len = len(memory)
        if memory:
            self._last_seen_fingerprint = self._fingerprint(memory[-1])

    def _poll(self):
        memory = self.client.get_memory(self.AGENT_NAME)
        if not memory:
            return

        if self._last_seen_fingerprint is not None:
            anchor = -1
            for i in range(len(memory) - 1, -1, -1):
                if self._fingerprint(memory[i]) == self._last_seen_fingerprint:
                    anchor = i
                    break
            if anchor < 0:
                self._sync_memory_stable(max_attempts=3, delay=0.3)
                return
            start = anchor + 1
        else:
            start = self._mem_len

        if start >= len(memory):
            return

        new_messages = memory[start:]
        self._mem_len = len(memory)
        self._last_seen_fingerprint = self._fingerprint(memory[-1])

        for msg in new_messages:
            if msg.get('role') != 'model':
                continue
            text = ''.join(p.get('text', '') for p in msg.get('parts', []))
            cmds = parse_commands(text)
            if not cmds:
                continue

            self.get_logger().info(f'[POLL] {len(cmds)} command(s) detected')
            feedback = self._dispatch(cmds)
            if feedback:
                self._execution_loop(feedback)
            self._sync_memory()
            return

    def _execution_loop(self, initial_feedback: str):
        feedback = initial_feedback
        while feedback and self._running:
            self.get_logger().info(f'[FEEDBACK -> ANUBIX] {feedback}')
            try:
                resp = self.client.chat(
                    prompt=feedback,
                    agent_name=self.AGENT_NAME,
                    engine=self.ENGINE)
            except OmniLinkAPIError as e:
                self.get_logger().error(f'[CHAT] {e.status_code}: {e.body}')
                break

            anubix_text = resp.get('text', '')
            if re.search(r'supervisor/force_stop', anubix_text, re.IGNORECASE):
                self.execute_command('force_stop')
                break

            cmds = parse_commands(anubix_text)
            if not cmds:
                self.get_logger().info('[DONE] No more commands - mission complete')
                break

            feedback = self._dispatch(cmds)

    def _dispatch(self, cmds: list) -> str:
        feedbacks = []
        for cmd_type, match in sorted(cmds, key=lambda x: CMD_PRIORITY.get(x[0], 99)):
            self.get_logger().info(f'[CMD] {match.group(0)}')
            fb = self._execute_one(cmd_type, match)
            if fb:
                feedbacks.append(fb)
            if cmd_type == 'force_stop':
                break
        return '\n'.join(feedbacks)

    def _execute_one(self, cmd_type: str, match: re.Match) -> Optional[str]:
        if cmd_type == 'force_stop':
            return self.execute_command('force_stop')
        if cmd_type == 'nav_goal_home':
            return self.execute_command('nav_goal_home')
        if cmd_type == 'nav_goal':
            return self.execute_command(
                'nav_goal', x=float(match.group(1)), y=float(match.group(2)))
        if cmd_type == 'target_camera':
            return self.execute_command(
                'target_camera', camera_number=int(match.group(1)))
        if cmd_type == 'perception_goal':
            return self.execute_command(
                'perception_goal', task_type=match.group(1).lower())
        if cmd_type == 'arm_nav_goal':
            return self.execute_command(
                'arm_nav_goal', signal=match.group(1).lower())
        if cmd_type == 'grip':
            return self.execute_command(
                'grip', action=match.group(1).lower() == 'true')
        if cmd_type == 'spectral_target':
            return self.execute_command(
                'spectral_target', task_type=match.group(1).lower())
        return None

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_pose_stamped(self, x: float, y: float, z: float = 0.0,
                           frame_id: str = 'map'):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        return ps


def main(args=None):
    rclpy.init(args=args)
    node = AnubixMasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
