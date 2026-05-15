#!/usr/bin/env python3
"""
ANUBIX ROS 2 Master Node
=========================
Bridges the ANUBIX OmniLink agent to four ROS 2 subsystems on the Jetson Orin Nano.

Uses a dedicated polling thread (not a ROS timer) so the executor is never blocked.
Subscription callbacks fire freely on the MultiThreadedExecutor while the poll
thread waits on feedback events.
"""

import os
import re
import sys
import time
import threading
import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
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

    DELEGATION_HIJACK_PATTERNS = (
        re.compile(r'\[Delegation to ["\']supervisor["\']', re.IGNORECASE),
        re.compile(r'Agent ["\']supervisor["\'] not found', re.IGNORECASE),
        re.compile(r'Ensure the agent profile exists', re.IGNORECASE),
    )

    def __init__(self):
        super().__init__('anubix_master')

        # Parameters
        self.declare_parameter('omni_key', '')
        self.declare_parameter('poll_interval', 3.0)
        self.declare_parameter('feedback_timeout', 120.0)
        self.declare_parameter('arm_home_x', 0.0)
        self.declare_parameter('arm_home_y', 0.0)
        self.declare_parameter('arm_home_z', 0.3)
        self.declare_parameter('robot_id', '34a957fd-d45c-4dbf-8e02-be8e1b5e349a')
        self.declare_parameter('task_id', '40e4060b-5bc8-4044-9d71-046fee27a757')

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
        self._task_id = self.get_parameter('task_id').value

        # OmniLink client
        self.get_logger().info(f'[INIT] Connecting to OmniLink with key: {omni_key[:12]}...')
        self.client = OmniLinkClient(omni_key=omni_key, timeout=120)
        self.AGENT_NAME = "ANUBIX"
        self.ENGINE = "g1-engine"

        # All subscriptions go into a reentrant group so they can fire
        # even while the poll thread is blocking on events.
        self._sub_group = ReentrantCallbackGroup()

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
        # force_stop is an edge-triggered emergency signal — strictly
        # VOLATILE so no publisher (master or rpi_bridge's e-stop)
        # latches state. Without this, two publishers each carry their
        # own latched value (TRANSIENT_LOCAL+depth=1 per publisher),
        # late-joining consumers receive both in nondeterministic
        # order, and a stale True from a previous rpi_bridge e-stop
        # session can lock every Jetson stack into "force_stopped" the
        # moment they boot. VOLATILE keeps the topic stateless: only
        # consumers alive at publish time see the signal, which is
        # exactly the right semantics for an emergency abort.
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers (supervisor command channels)
        self.pub_nav_goal = self.create_publisher(PoseStamped, '/supervisor/nav_goal', cmd_qos)
        # Companion flag: True → nav must stop ~1 m short of the target so
        # the on-board camera can take over the final approach. False → nav
        # drives all the way to the goal. Latched (TRANSIENT_LOCAL) so the
        # nav node always sees the most recent value.
        self.pub_nav_vision = self.create_publisher(Bool, '/supervisor/nav_vision', cmd_qos)
        self.pub_perception = self.create_publisher(String, '/supervisor/perception_goal', cmd_qos)
        self.pub_target_camera = self.create_publisher(String, '/supervisor/target_camera', cmd_qos)
        self.pub_arm_nav_goal = self.create_publisher(PoseStamped, '/supervisor/arm_nav_goal', cmd_qos)
        self.pub_grip = self.create_publisher(Bool, '/supervisor/grip', cmd_qos)
        self.pub_spectral = self.create_publisher(String, '/supervisor/spectral_target', cmd_qos)
        self.pub_force_stop = self.create_publisher(Bool, '/supervisor/force_stop', force_stop_qos)
        self.get_logger().info('[INIT] All 8 supervisor publishers created')

        # Feedback synchronization events
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

        # Subscribers (feedback from stacks) — use reentrant callback group
        self.create_subscription(
            String, '/nav/status', self._cb_nav, sub_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            String, '/perception/status', self._cb_perception, sub_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            String, '/arm/arm_status', self._cb_arm, sub_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            String, '/arm/gripper_status', self._cb_gripper, sub_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/arm/touch_status', self._cb_touch, sub_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            String, '/spectrometer/status', self._cb_spectro, sub_qos,
            callback_group=self._sub_group)
        self.create_subscription(
            Pose, '/perception/target_pose', self._cb_target_pose, sub_qos,
            callback_group=self._sub_group)
        self.get_logger().info('[INIT] All 7 feedback subscribers created')

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
        self._poll_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ANUBIX ROS 2 Master Node - Jetson Orin Nano')
        self.get_logger().info('  4-stack architecture: NAV | PERCEPTION | ARM | SPECTRO')
        self.get_logger().info(f'  Poll interval: {self.poll_interval}s')
        self.get_logger().info(f'  Feedback timeout: {self.feedback_timeout}s')
        self.get_logger().info('=' * 60)

        # Stabilize memory cursor
        self.get_logger().info('[INIT] Stabilizing OmniLink memory cursor...')
        self._sync_memory_stable()
        self.get_logger().info(
            f'[READY] Listening on agent "{self.AGENT_NAME}" '
            f'(memory at {self._mem_len} msgs, fingerprint locked)')
        self.get_logger().info(
            '  Fire a task via the OmniLink web UI. Press Ctrl+C to stop.')

        # Start the polling thread (NOT a ROS timer — avoids executor deadlock)
        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True, name='omnilink_poll')
        self._poll_thread.start()
        self.get_logger().info('[INIT] OmniLink poll thread started')

    # ── Subscriber callbacks (fire on executor threads — never blocked) ───────

    def _cb_nav(self, msg):
        self._fb_nav = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /nav/status = "{self._fb_nav}"')
        # Only set event for terminal statuses, not "navigating"
        if self._fb_nav in ('point_reached', 'blocked', 'failure'):
            self._ev_nav.set()

    def _cb_perception(self, msg):
        self._fb_perception = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /perception/status = "{self._fb_perception}"')
        self._ev_perception.set()

    def _cb_arm(self, msg):
        self._fb_arm = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /arm/arm_status = "{self._fb_arm}"')
        self._ev_arm.set()

    def _cb_gripper(self, msg):
        self._fb_gripper = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /arm/gripper_status = "{self._fb_gripper}"')
        self._ev_gripper.set()

    def _cb_touch(self, msg):
        self._fb_touch = bool(msg.data)
        self.get_logger().info(f'[RX] /arm/touch_status = {self._fb_touch}')
        self._ev_touch.set()

    def _cb_spectro(self, msg):
        self._fb_spectro = (msg.data or '').strip().lower()
        self.get_logger().info(f'[RX] /spectrometer/status = "{self._fb_spectro}"')
        if self._fb_spectro in ('success', 'failure'):
            self._ev_spectro.set()

    def _cb_target_pose(self, msg):
        with self._target_pose_lock:
            self._latest_target_pose = msg
        self.get_logger().info(
            f'[RX] /perception/target_pose = '
            f'({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})')

    # ── Command execution ─────────────────────────────────────────────────────

    def execute_command(self, cmd_type: str, **kwargs) -> Optional[str]:
        if self._force_stopped and cmd_type != 'force_stop':
            self.get_logger().warning(
                f'[EXEC] Ignoring "{cmd_type}" - robot is force_stopped')
            return None

        handler = getattr(self, f'_do_{cmd_type}', None)
        if handler is None:
            self.get_logger().error(f'[EXEC] No handler for command: "{cmd_type}"')
            return None

        self.get_logger().info(f'[EXEC] Executing: {cmd_type} kwargs={kwargs}')
        try:
            result = handler(**kwargs)
            self.get_logger().info(f'[EXEC] Result: {result}')
            return result
        except Exception as e:
            self.get_logger().error(
                f'[EXEC] Exception in handler "{cmd_type}": {e}\n'
                f'{traceback.format_exc()}')
            return None

    def _do_force_stop(self) -> str:
        # Edge-triggered abort: publish True so every consumer aborts
        # whatever it is doing, wait long enough for the True to land
        # and for callbacks to flip their _force_stopped flag, then
        # publish False to re-arm consumers for the NEXT command. The
        # topic is VOLATILE so no value is latched anywhere — late
        # joiners never inherit a stale True, which is exactly the
        # right semantics for an emergency signal.
        self.pub_force_stop.publish(Bool(data=True))
        self._force_stopped = True
        self._mission_active = False
        self.get_logger().warning('*** FORCE STOP PUBLISHED ***')

        time.sleep(0.2)

        self.pub_force_stop.publish(Bool(data=False))
        self._force_stopped = False
        self.get_logger().info(
            '[FORCE_STOP] /supervisor/force_stop re-armed — '
            'consumers ready for the next command')
        return '/system/status: force_stopped'

    def _do_nav_goal(self, x: float, y: float, vision: bool = False) -> str:
        ps = self._make_pose_stamped(x, y, 0.0)
        self._ev_nav.clear()
        self._fb_nav = None
        # Publish the vision flag BEFORE the goal so the nav node has it
        # available the moment the pose arrives.
        self.pub_nav_vision.publish(Bool(data=bool(vision)))
        self.pub_nav_goal.publish(ps)
        self.get_logger().info(
            f'[TX] /supervisor/nav_goal ({x:.2f}, {y:.2f}) vision={vision}')
        self.get_logger().info(
            f'[WAIT] Waiting for /nav/status (timeout={self.feedback_timeout}s)...')
        if not self._ev_nav.wait(self.feedback_timeout):
            self.get_logger().error(
                '[TIMEOUT] /nav/status never received! '
                'Check: RPi nav_node running, DDS config matching, '
                'same ROS_DOMAIN_ID, network connectivity.')
            return '/nav/status: failure'
        self._position = (x, y)
        self._mission_active = True
        return f'/nav/status: {self._fb_nav}'

    def _do_nav_goal_home(self) -> str:
        self.get_logger().info(
            f'[TX] nav_goal_home -> ({self._home[0]}, {self._home[1]})')
        return self._do_nav_goal(self._home[0], self._home[1], vision=False)

    def _do_target_camera(self, camera_number: int) -> None:
        self.pub_target_camera.publish(String(data=str(camera_number)))
        self._current_camera = camera_number
        self.get_logger().info(f'[TX] /supervisor/target_camera = {camera_number}')
        return None

    def _do_perception_goal(self, task_type: str) -> str:
        self._ev_perception.clear()
        self._fb_perception = None
        self.pub_perception.publish(String(data=task_type))
        self._current_task = task_type
        self.get_logger().info(f'[TX] /supervisor/perception_goal = "{task_type}"')
        self.get_logger().info(
            f'[WAIT] Waiting for /perception/status (timeout={self.feedback_timeout}s)...')
        if not self._ev_perception.wait(self.feedback_timeout):
            self.get_logger().error(
                '[TIMEOUT] /perception/status never received! '
                'Check: vision_node running on Jetson, or perception_node on RPi.')
            return '/perception/status: not_found'
        return f'/perception/status: {self._fb_perception}'

    def _do_arm_nav_goal(self, signal: str) -> str:
        target_is_home = (signal == 'home') or self._post_release_retract

        if target_is_home:
            ps = self._make_pose_stamped(
                *self.arm_home_pose, frame_id='base_link')
            if signal == 'move' and self._post_release_retract:
                self.get_logger().info(
                    '[TX] Post-release retract: overriding move -> home pose')
        elif signal == 'move':
            with self._target_pose_lock:
                tgt = self._latest_target_pose
            if tgt is None:
                # Mocking phase: still publish a goal so the arm node can
                # report success and the pipeline stays testable. Use the
                # configured home pose as a placeholder target.
                self.get_logger().warning(
                    '[TX] arm_nav_goal_move with no /perception/target_pose '
                    'yet — forwarding home pose as placeholder so the arm '
                    'mock can complete. Hook up vision before going live.')
                ps = self._make_pose_stamped(
                    *self.arm_home_pose, frame_id='base_link')
            else:
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'base_link'
                ps.pose = tgt
        else:
            self.get_logger().error(f'[TX] Unknown arm signal: "{signal}"')
            return '/arm/arm_status: mechanical_error'

        self._ev_arm.clear()
        self._fb_arm = None
        self.pub_arm_nav_goal.publish(ps)
        self.get_logger().info(
            f'[TX] /supervisor/arm_nav_goal (signal={signal}, '
            f'dest={"home" if target_is_home else "target"})')
        self.get_logger().info(
            f'[WAIT] Waiting for /arm/arm_status (timeout={self.feedback_timeout}s)...')
        if not self._ev_arm.wait(self.feedback_timeout):
            self.get_logger().error(
                '[TIMEOUT] /arm/arm_status never received! '
                'Check: arm_node running on Jetson.')
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
        self.get_logger().info(
            f'[TX] /supervisor/grip = {action} ({"close" if action else "open"})')
        self.get_logger().info(
            f'[WAIT] Waiting for /arm/gripper_status (timeout={self.feedback_timeout}s)...')

        if not self._ev_gripper.wait(self.feedback_timeout):
            self.get_logger().error(
                '[TIMEOUT] /arm/gripper_status never received! '
                'Check: arm_node running.')
            return '/arm/gripper_status: mechanical_error'
        gripper_line = f'/arm/gripper_status: {self._fb_gripper}'

        if not action:
            self._gripping = False
            if self._fb_gripper == 'successful_release':
                self._post_release_retract = True
                self.get_logger().info(
                    '[STATE] Post-release retract armed: next arm move -> home')
            return gripper_line

        self.get_logger().info(
            f'[WAIT] Waiting for /arm/touch_status (timeout={self.feedback_timeout}s)...')
        if not self._ev_touch.wait(self.feedback_timeout):
            self.get_logger().warning(
                '[TIMEOUT] /arm/touch_status not received - assuming no touch')
            touch_line = '/arm/touch_status: false'
        else:
            touch_line = f'/arm/touch_status: {"true" if self._fb_touch else "false"}'
        self._gripping = (
            bool(self._fb_touch) and self._fb_gripper == 'successful_grip')
        return f'{gripper_line}\n{touch_line}'

    def _do_spectral_target(self, task_type: str,
                            robot_id: str = '', task_id: str = '') -> str:
        # OmniLink supplies robot_id/task_id with each spectrometer call.
        # We forward them verbatim so the Supabase uploader can attribute
        # the reading to the correct robot/task without hardcoded UUIDs.
        rid = (robot_id or self._robot_id or '').strip()
        tid = (task_id or self._task_id or '').strip()
        payload = task_type
        if rid or tid:
            payload = f'{task_type}|{rid}|{tid}'

        self._ev_spectro.clear()
        self._fb_spectro = None
        self.pub_spectral.publish(String(data=payload))
        self.get_logger().info(
            f'[TX] /supervisor/spectral_target = "{payload}"  '
            f'(task={task_type!r} robot_id={rid!r} task_id={tid!r})')
        self.get_logger().info(
            f'[WAIT] Waiting for /spectrometer/status (timeout={self.feedback_timeout}s)...')
        if not self._ev_spectro.wait(self.feedback_timeout):
            self.get_logger().error(
                '[TIMEOUT] /spectrometer/status never received! '
                'Check: spectrometer_node running on Jetson.')
            return '/spectrometer/status: failure'
        feedback = f'/spectrometer/status: {self._fb_spectro}'
        if self._fb_spectro == 'success':
            feedback += f'\nrobot_id: {rid}\ntask_id: {tid}'
        return feedback

    # ── OmniLink polling (runs in dedicated thread — NOT a ROS timer) ─────────

    def _poll_loop(self):
        self.get_logger().info('[POLL] Poll loop started')
        while self._running and rclpy.ok():
            try:
                self._poll_count += 1
                self._poll()
            except OmniLinkAPIError as e:
                self.get_logger().error(
                    f'[POLL] OmniLink API error: HTTP {e.status_code}: {e.body}')
            except Exception as e:
                self.get_logger().error(
                    f'[POLL] Unexpected error in poll: {e}\n'
                    f'{traceback.format_exc()}')
            time.sleep(self.poll_interval)
        self.get_logger().info('[POLL] Poll loop exited')

    @staticmethod
    def _fingerprint(msg: dict) -> str:
        role = msg.get('role', '')
        text = ''.join(p.get('text', '') for p in msg.get('parts', []))
        return f'{role}::{text}'

    def _sync_memory_stable(self, max_attempts: int = 8, delay: float = 0.5):
        prev_len = -1
        prev_fp = None
        for attempt in range(max_attempts):
            try:
                memory = self.client.get_memory(self.AGENT_NAME) or []
            except Exception as e:
                self.get_logger().error(
                    f'[MEM] get_memory failed (attempt {attempt+1}): {e}')
                time.sleep(delay)
                continue
            cur_len = len(memory)
            cur_fp = self._fingerprint(memory[-1]) if memory else None
            if cur_len == prev_len and cur_fp == prev_fp:
                self._mem_len = cur_len
                self._last_seen_fingerprint = cur_fp
                self.get_logger().info(
                    f'[MEM] Cursor stable at {cur_len} msgs (attempt {attempt+1})')
                return
            prev_len, prev_fp = cur_len, cur_fp
            time.sleep(delay)
        self._mem_len = prev_len if prev_len >= 0 else 0
        self._last_seen_fingerprint = prev_fp
        self.get_logger().warning(
            f'[MEM] Cursor not fully stable; locked at {self._mem_len} msgs')

    def _sync_memory(self):
        try:
            memory = self.client.get_memory(self.AGENT_NAME) or []
            self._mem_len = len(memory)
            if memory:
                self._last_seen_fingerprint = self._fingerprint(memory[-1])
        except Exception as e:
            self.get_logger().error(f'[MEM] sync_memory failed: {e}')

    def _poll(self):
        if self._poll_count % 10 == 1:
            self.get_logger().debug(
                f'[POLL] Tick #{self._poll_count} — checking OmniLink memory...')

        try:
            memory = self.client.get_memory(self.AGENT_NAME)
        except Exception as e:
            self.get_logger().error(
                f'[POLL] get_memory() failed: {e}')
            raise

        if not memory:
            self.get_logger().warning('[POLL] get_memory returned empty/None')
            return

        if self._last_seen_fingerprint is not None:
            anchor = -1
            for i in range(len(memory) - 1, -1, -1):
                if self._fingerprint(memory[i]) == self._last_seen_fingerprint:
                    anchor = i
                    break
            if anchor < 0:
                self.get_logger().warning(
                    '[POLL] Last-seen fingerprint not found in memory snapshot! '
                    'Possible OmniLink server cache issue. Re-stabilizing...')
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

        self.get_logger().info(
            f'[POLL] {len(new_messages)} new message(s) in memory')

        for msg in new_messages:
            role = msg.get('role', '?')
            text = ''.join(p.get('text', '') for p in msg.get('parts', []))

            if role != 'model':
                self.get_logger().debug(
                    f'[POLL] Skipping non-model message (role={role})')
                continue

            self.get_logger().info(
                f'[POLL] Processing model message ({len(text)} chars): '
                f'"{text[:120]}..."')

            cmds = parse_commands(text)

            # Delegation-pipeline recovery
            if not cmds and self._is_delegation_hijack(text):
                self.get_logger().warning(
                    '[POLL] Delegation pipeline hijacked the response! '
                    'OmniLink tried to route "supervisor/" as an agent name. '
                    'Attempting recovery via re-prompt...')
                recovered = self._recover_from_delegation()
                if recovered:
                    text = recovered
                    cmds = parse_commands(text)
                else:
                    self.get_logger().error(
                        '[POLL] Delegation recovery FAILED - commands lost')

            if not cmds:
                self.get_logger().info(
                    '[POLL] No supervisor commands found in this message')
                continue

            self.get_logger().info(
                f'[POLL] >>> {len(cmds)} command(s) detected: '
                f'{[m.group(0) for _, m in cmds]}')

            feedback = self._dispatch(cmds)
            if feedback:
                self._execution_loop(feedback)
            self._sync_memory()
            return  # one batch per poll tick

    # ── Delegation-hijack recovery ────────────────────────────────────────────

    def _is_delegation_hijack(self, text: str) -> bool:
        return any(p.search(text) for p in self.DELEGATION_HIJACK_PATTERNS)

    def _recover_from_delegation(self, max_attempts: int = 2) -> Optional[str]:
        prompts = [
            "SYSTEM NOTE: your previous response was intercepted by the platform's "
            "delegation pipeline because it parsed `supervisor/` as an agent name. "
            "There is no agent called 'supervisor' – it is a literal command prefix. "
            "Re-emit the same step's command(s) as PLAIN TEXT in your reply, exactly "
            "as written in your custom instructions (e.g. `supervisor/nav_goal_3_5`). "
            "Do not delegate. Do not call any tools.",
            "Please respond again with the supervisor/* command(s) for the current "
            "step written verbatim as plain text. No tool calls, no delegation.",
        ]
        for i in range(max_attempts):
            prompt = prompts[min(i, len(prompts) - 1)]
            try:
                self.get_logger().info(
                    f'[RECOVER] Attempt {i+1}/{max_attempts} — re-prompting...')
                resp = self.client.chat(
                    prompt=prompt,
                    agent_name=self.AGENT_NAME,
                    engine=self.ENGINE,
                )
            except OmniLinkAPIError as e:
                self.get_logger().error(
                    f'[RECOVER] API error: {e.status_code}: {e.body}')
                return None
            except Exception as e:
                self.get_logger().error(f'[RECOVER] Exception: {e}')
                return None

            text = resp.get('text', '') or ''
            self.get_logger().info(
                f'[RECOVER] Response ({len(text)} chars): "{text[:120]}..."')

            if self._is_delegation_hijack(text):
                self.get_logger().warning(
                    f'[RECOVER] Attempt {i+1} still hijacked')
                continue
            if parse_commands(text):
                self.get_logger().info(
                    f'[RECOVER] Attempt {i+1} returned valid commands!')
                return text

        self.get_logger().error(
            '[RECOVER] All attempts failed - could not get non-hijacked response')
        return None

    # ── Execution loop ────────────────────────────────────────────────────────

    def _execution_loop(self, initial_feedback: str):
        feedback = initial_feedback
        loop_iter = 0
        while feedback and self._running:
            loop_iter += 1
            self.get_logger().info(
                f'\n{"─"*55}\n'
                f'[FEEDBACK -> ANUBIX] (iteration {loop_iter})\n'
                f'{feedback}\n'
                f'{"─"*55}')

            try:
                resp = self.client.chat(
                    prompt=feedback,
                    agent_name=self.AGENT_NAME,
                    engine=self.ENGINE)
            except OmniLinkAPIError as e:
                self.get_logger().error(
                    f'[LOOP] OmniLink chat error: {e.status_code}: {e.body}')
                break
            except Exception as e:
                self.get_logger().error(
                    f'[LOOP] Exception during chat: {e}\n'
                    f'{traceback.format_exc()}')
                break

            anubix_text = resp.get('text', '')
            self.get_logger().info(
                f'[LOOP] ANUBIX response ({len(anubix_text)} chars): '
                f'"{anubix_text[:200]}..."')

            # Delegation hijack recovery mid-loop
            if not parse_commands(anubix_text) and self._is_delegation_hijack(anubix_text):
                self.get_logger().warning(
                    '[LOOP] Mid-mission delegation hijack — recovering')
                recovered = self._recover_from_delegation()
                if recovered:
                    anubix_text = recovered

            if re.search(r'supervisor/force_stop', anubix_text, re.IGNORECASE):
                self.get_logger().warning(
                    '[LOOP] force_stop detected — aborting mission')
                self.execute_command('force_stop')
                break

            cmds = parse_commands(anubix_text)
            if not cmds:
                self.get_logger().info(
                    '[LOOP] No more commands — mission step complete')
                break

            self.get_logger().info(
                f'[LOOP] {len(cmds)} command(s): '
                f'{[m.group(0) for _, m in cmds]}')

            feedback = self._dispatch(cmds)
            if not feedback:
                self.get_logger().warning(
                    '[LOOP] Commands produced no feedback — yielding to poll')
                break

    # ── Dispatch ──────────────────────────────────────────────────────────────

    def _dispatch(self, cmds: list) -> str:
        feedbacks = []
        for cmd_type, match in sorted(
                cmds, key=lambda x: CMD_PRIORITY.get(x[0], 99)):
            self.get_logger().info(f'[CMD] >>> {match.group(0)}')
            fb = self._execute_one(cmd_type, match)
            if fb:
                self.get_logger().info(
                    f'[CMD] <<< {fb.replace(chr(10), " | ")}')
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
            vision_flag_str = (match.group(3) or '').lower() if match.lastindex and match.lastindex >= 3 else ''
            vision_flag = vision_flag_str == 'true'
            return self.execute_command(
                'nav_goal',
                x=float(match.group(1)),
                y=float(match.group(2)),
                vision=vision_flag)
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
                'spectral_target',
                task_type=match.group(1).lower(),
                robot_id=(match.group(2) or '') if match.lastindex and match.lastindex >= 2 else '',
                task_id=(match.group(3) or '') if match.lastindex and match.lastindex >= 3 else '')
        self.get_logger().error(f'[DISPATCH] Unknown command type: "{cmd_type}"')
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

    # MultiThreadedExecutor: subscription callbacks run on separate threads
    # so they fire even while the poll thread blocks on event.wait().
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('[SHUTDOWN] Ctrl+C received')
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
