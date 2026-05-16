#!/usr/bin/env python3
"""
ANUBIX ROS 2 Master Node - Clean Rebuild
==========================================
Bridges ANUBIX OmniLink agent to four ROS 2 subsystems on Jetson + RPi.

Architecture:
- Polls OmniLink memory for new model messages containing supervisor/* commands
- Executes commands ONE AT A TIME (sequential architecture, no batch dispatch)
- Publishes to appropriate ROS 2 command topics
- Waits for feedback from status topics
- Sends formatted feedback back to OmniLink agent
- Agent waits for confirmation before sending next command

Command Flow:
  Agent → supervisor/robot_id_xxx → Master publishes /supervisor/robot_id
                                   → Returns /context/robot_id: xxx
  Agent receives confirmation
  Agent → supervisor/nav_vision_true → Master publishes /supervisor/nav_vision
                                      → Returns /supervisor/nav_vision: true
  Agent receives confirmation
  Agent → supervisor/nav_goal_40_45 → Master publishes /supervisor/nav_goal
                                     → Waits for /nav/status
                                     → Returns /nav/status: point_reached

Key Features:
- Fingerprint-based memory tracking (never replays history)
- Delegation-hijack recovery (handles OmniLink routing "supervisor/" as agent name)
- Clean separation between OmniLink polling and ROS execution
- Proper QoS profiles for latched commands vs streaming feedback
- Sequential execution (one command at a time, no race conditions)
"""

import os
import re
import sys
import time
import logging
import argparse
import threading
from typing import Optional, Tuple

try:
    from omnilink.client import OmniLinkClient, OmniLinkAPIError
except ImportError:
    print("ERROR: omnilink not installed. Run: pip install omnilink")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose


# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("anubix_master")


# ─────────────────────────────────────────────────────────────────────────────
# Command Parser
# ─────────────────────────────────────────────────────────────────────────────

_UUID_FRAG = r'[A-Za-z0-9\-]{8,}'

CMD_PATTERNS = [
    ("force_stop", re.compile(r'supervisor/force_stop', re.IGNORECASE)),
    ("robot_id", re.compile(r'supervisor/robot_id_(' + _UUID_FRAG + r')', re.IGNORECASE)),
    ("task_id", re.compile(r'supervisor/task_id_(' + _UUID_FRAG + r')', re.IGNORECASE)),
    ("nav_goal_home", re.compile(r'supervisor/nav_goal_home', re.IGNORECASE)),
    ("nav_vision", re.compile(r'supervisor/nav_vision_(true|false)', re.IGNORECASE)),
    ("nav_goal", re.compile(r'supervisor/nav_goal_([-\d]+(?:\.\d+)?)_([-\d]+(?:\.\d+)?)', re.IGNORECASE)),
    ("target_camera", re.compile(r'supervisor/target_camera_(\d+)', re.IGNORECASE)),
    ("perception_goal", re.compile(r'supervisor/perception_goal_([a-z][a-z_]*)', re.IGNORECASE)),
    ("arm_nav_goal", re.compile(r'supervisor/arm_nav_goal_([a-z]+)', re.IGNORECASE)),
    ("grip", re.compile(r'supervisor/grip_(true|false)', re.IGNORECASE)),
    ("spectral_target", re.compile(
        r'supervisor/spectral_target_([a-z][a-z_]*)'
        r'(?:\|(' + _UUID_FRAG + r'))?'
        r'(?:\|(' + _UUID_FRAG + r'))?',
        re.IGNORECASE)),
]


def parse_commands(text: str):
    """Extract all supervisor/* commands from agent text response."""
    results = []
    for cmd_type, pattern in CMD_PATTERNS:
        for match in pattern.finditer(text):
            results.append((cmd_type, match))
    return results


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Bridge
# ─────────────────────────────────────────────────────────────────────────────

class AnubixROSBridge(Node):
    """
    ROS 2 node that publishes supervisor commands and waits for feedback.

    Each execute() call:
    1. Publishes the command to the appropriate supervisor/* topic
    2. Blocks until feedback arrives on the corresponding status topic
    3. Returns formatted feedback string for OmniLink agent

    Runs a background executor thread so callbacks can fire while main thread
    is polling OmniLink.
    """

    def __init__(self,
                 feedback_timeout: float = 120.0,
                 arm_home_pose: Tuple[float, float, float] = (0.0, 0.0, 0.3),
                 robot_id: str = '',
                 task_id: str = ''):
        super().__init__('anubix_master')

        self.feedback_timeout = feedback_timeout
        self.arm_home_pose = arm_home_pose

        # Context IDs (can be overridden by robot_id/task_id commands)
        self._context_robot_id = robot_id
        self._context_task_id = task_id

        # State tracking
        self._force_stopped = False
        self._latest_target_pose: Optional[Pose] = None
        self._target_pose_lock = threading.Lock()
        self._post_release_retract = False

        # Reentrant callback group for all subscriptions
        self._sub_group = ReentrantCallbackGroup()

        # QoS profiles
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Latched
        )
        trigger_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            durability=DurabilityPolicy.VOLATILE,  # Not latched
        )
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,  # Edge-triggered
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.pub_robot_id = self.create_publisher(String, '/supervisor/robot_id', cmd_qos)
        self.pub_task_id = self.create_publisher(String, '/supervisor/task_id', cmd_qos)
        self.pub_nav_vision = self.create_publisher(Bool, '/supervisor/nav_vision', cmd_qos)
        self.pub_nav_goal = self.create_publisher(PoseStamped, '/supervisor/nav_goal', cmd_qos)
        self.pub_target_camera = self.create_publisher(String, '/supervisor/target_camera', cmd_qos)
        self.pub_perception = self.create_publisher(String, '/supervisor/perception_goal', trigger_qos)
        self.pub_arm_nav_goal = self.create_publisher(PoseStamped, '/supervisor/arm_nav_goal', cmd_qos)
        self.pub_grip = self.create_publisher(Bool, '/supervisor/grip', cmd_qos)
        self.pub_spectral = self.create_publisher(String, '/supervisor/spectral_target', cmd_qos)
        self.pub_force_stop = self.create_publisher(Bool, '/supervisor/force_stop', force_stop_qos)

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

        # Subscribers
        self.create_subscription(
            String, '/nav/status', self._cb_nav, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/perception/status', self._cb_perception, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/arm/arm_status', self._cb_arm, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/arm/gripper_status', self._cb_gripper, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/arm/touch_status', self._cb_touch, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/spectrometer/status', self._cb_spectro, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            Pose, '/perception/target_pose', self._cb_target_pose, sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._cb_force_stop, force_stop_qos, callback_group=self._sub_group)

        log.info("[ROS2] AnubixROSBridge initialized")
        log.info(f"       feedback_timeout={feedback_timeout}s")
        log.info(f"       arm_home_pose={arm_home_pose}")

    # ── Subscriber Callbacks ──────────────────────────────────────────────────

    def _cb_nav(self, msg):
        self._fb_nav = (msg.data or '').strip().lower()
        log.info(f"[RX] /nav/status = '{self._fb_nav}'")
        if self._fb_nav in ('point_reached', 'blocked', 'failure'):
            self._ev_nav.set()

    def _cb_perception(self, msg):
        self._fb_perception = (msg.data or '').strip().lower()
        log.info(f"[RX] /perception/status = '{self._fb_perception}'")
        self._ev_perception.set()

    def _cb_arm(self, msg):
        self._fb_arm = (msg.data or '').strip().lower()
        log.info(f"[RX] /arm/arm_status = '{self._fb_arm}'")
        self._ev_arm.set()

    def _cb_gripper(self, msg):
        self._fb_gripper = (msg.data or '').strip().lower()
        log.info(f"[RX] /arm/gripper_status = '{self._fb_gripper}'")
        self._ev_gripper.set()

    def _cb_touch(self, msg):
        self._fb_touch = bool(msg.data)
        log.info(f"[RX] /arm/touch_status = {self._fb_touch}")
        self._ev_touch.set()

    def _cb_spectro(self, msg):
        self._fb_spectro = (msg.data or '').strip().lower()
        log.info(f"[RX] /spectrometer/status = '{self._fb_spectro}'")
        if self._fb_spectro in ('success', 'failure'):
            self._ev_spectro.set()

    def _cb_target_pose(self, msg):
        with self._target_pose_lock:
            self._latest_target_pose = msg
        log.info(f"[RX] /perception/target_pose = ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})")

    def _cb_force_stop(self, msg):
        if bool(msg.data):
            self._force_stopped = True
            log.warning("[RX] Force stop received!")
        else:
            self._force_stopped = False
            log.info("[RX] Force stop cleared")

    # ── Public API (called by OmniLink master) ────────────────────────────────

    def execute(self, cmd_type: str, **kwargs) -> Optional[str]:
        """Execute a single command and return feedback string."""
        if self._force_stopped and cmd_type != 'force_stop':
            log.warning(f"[EXEC] Ignoring '{cmd_type}' - robot is force_stopped")
            return None

        handler = getattr(self, f'_do_{cmd_type}', None)
        if handler is None:
            log.error(f"[EXEC] No handler for '{cmd_type}'")
            return None

        try:
            return handler(**kwargs)
        except Exception as e:
            log.error(f"[EXEC] Exception in '{cmd_type}': {e}")
            return None

    # ── Command Handlers ──────────────────────────────────────────────────────

    def _do_force_stop(self) -> str:
        self.pub_force_stop.publish(Bool(data=True))
        self._force_stopped = True
        log.warning("[TX] /supervisor/force_stop = TRUE")
        time.sleep(0.2)
        self.pub_force_stop.publish(Bool(data=False))
        self._force_stopped = False
        log.info("[TX] /supervisor/force_stop = FALSE (re-armed)")
        return '/system/status: force_stopped'

    def _do_robot_id(self, robot_id: str) -> str:
        self._context_robot_id = robot_id
        self.pub_robot_id.publish(String(data=robot_id))
        log.info(f"[TX] /supervisor/robot_id = '{robot_id}'")
        time.sleep(0.05)  # DDS propagation delay
        return f'/context/robot_id: {robot_id}'

    def _do_task_id(self, task_id: str) -> str:
        self._context_task_id = task_id
        self.pub_task_id.publish(String(data=task_id))
        log.info(f"[TX] /supervisor/task_id = '{task_id}'")
        time.sleep(0.05)  # DDS propagation delay
        return f'/context/task_id: {task_id}'

    def _do_nav_vision(self, vision: bool) -> str:
        self.pub_nav_vision.publish(Bool(data=vision))
        log.info(f"[TX] /supervisor/nav_vision = {vision}")
        time.sleep(0.1)  # DDS propagation delay
        return f'/supervisor/nav_vision: {vision}'

    def _do_nav_goal(self, x: float, y: float) -> str:
        ps = self._make_pose_stamped(x, y, 0.0)
        self._ev_nav.clear()
        self._fb_nav = None

        # Publish IDs alongside nav_goal (for nav_node on RPi)
        if self._context_robot_id:
            self.pub_robot_id.publish(String(data=self._context_robot_id))
        if self._context_task_id:
            self.pub_task_id.publish(String(data=self._context_task_id))

        self.pub_nav_goal.publish(ps)
        log.info(f"[TX] /supervisor/nav_goal = ({x:.2f}, {y:.2f}) "
                f"robot_id='{self._context_robot_id or 'none'}' "
                f"task_id='{self._context_task_id or 'none'}'")

        if not self._ev_nav.wait(self.feedback_timeout):
            log.error(f"[TIMEOUT] No /nav/status received in {self.feedback_timeout}s")
            return '/nav/status: failure'

        return f'/nav/status: {self._fb_nav}'

    def _do_nav_goal_home(self) -> str:
        return self._do_nav_goal(0.0, 0.0)

    def _do_target_camera(self, camera_number: int) -> str:
        self.pub_target_camera.publish(String(data=str(camera_number)))
        log.info(f"[TX] /supervisor/target_camera = {camera_number}")
        time.sleep(0.05)  # DDS propagation delay
        return f'/supervisor/target_camera: {camera_number}'

    def _do_perception_goal(self, task_type: str) -> str:
        self._ev_perception.clear()
        self._fb_perception = None
        self.pub_perception.publish(String(data=task_type))
        log.info(f"[TX] /supervisor/perception_goal = '{task_type}'")

        if not self._ev_perception.wait(self.feedback_timeout):
            log.error(f"[TIMEOUT] No /perception/status received in {self.feedback_timeout}s")
            return '/perception/status: not_found'

        return f'/perception/status: {self._fb_perception}'

    def _do_arm_nav_goal(self, signal: str) -> str:
        target_is_home = (signal == 'home') or self._post_release_retract

        if target_is_home:
            ps = self._make_pose_stamped(*self.arm_home_pose, frame_id='base_link')
            if signal == 'move' and self._post_release_retract:
                log.info("[ARM] Post-release retract: overriding 'move' with home pose")
        elif signal == 'move':
            with self._target_pose_lock:
                tgt = self._latest_target_pose
            if tgt is None:
                log.error("[ARM] arm_nav_goal_move but no target_pose received yet!")
                return '/arm/arm_status: mechanical_error'
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'base_link'
            ps.pose = tgt
        else:
            log.error(f"[ARM] Unknown signal '{signal}'")
            return '/arm/arm_status: mechanical_error'

        self._ev_arm.clear()
        self._fb_arm = None
        self.pub_arm_nav_goal.publish(ps)
        log.info(f"[TX] /supervisor/arm_nav_goal (signal={signal}, dest={'home' if target_is_home else 'target'})")

        if not self._ev_arm.wait(self.feedback_timeout):
            log.error(f"[TIMEOUT] No /arm/arm_status received in {self.feedback_timeout}s")
            return '/arm/arm_status: mechanical_error'

        if target_is_home and self._fb_arm == 'success':
            self._post_release_retract = False

        return f'/arm/arm_status: {self._fb_arm}'

    def _do_grip(self, action: bool) -> str:
        self._ev_gripper.clear()
        self._ev_touch.clear()
        self._fb_gripper = None
        self._fb_touch = None
        self.pub_grip.publish(Bool(data=action))
        log.info(f"[TX] /supervisor/grip = {action}")

        if not self._ev_gripper.wait(self.feedback_timeout):
            log.error(f"[TIMEOUT] No /arm/gripper_status received in {self.feedback_timeout}s")
            return '/arm/gripper_status: mechanical_error'

        gripper_line = f'/arm/gripper_status: {self._fb_gripper}'

        if not action:
            # Release - arm post-release retract guard
            if self._fb_gripper == 'successful_release':
                self._post_release_retract = True
                log.info("[GRIP] Post-release retract armed")
            return gripper_line

        # Close - wait for touch sensor
        if not self._ev_touch.wait(self.feedback_timeout):
            log.warning("[TIMEOUT] No /arm/touch_status received, assuming false")
            touch_line = '/arm/touch_status: false'
        else:
            touch_line = f'/arm/touch_status: {"true" if self._fb_touch else "false"}'

        return f'{gripper_line}\n{touch_line}'

    def _do_spectral_target(self, task_type: str, robot_id: str = '', task_id: str = '') -> str:
        # Use context IDs if not explicitly provided
        rid = robot_id or self._context_robot_id or ''
        tid = task_id or self._context_task_id or ''

        payload = task_type
        if rid or tid:
            payload = f'{task_type}|{rid}|{tid}'

        self._ev_spectro.clear()
        self._fb_spectro = None
        self.pub_spectral.publish(String(data=payload))
        log.info(f"[TX] /supervisor/spectral_target = '{payload}'")

        if not self._ev_spectro.wait(self.feedback_timeout):
            log.error(f"[TIMEOUT] No /spectrometer/status received in {self.feedback_timeout}s")
            return '/spectrometer/status: failure'

        return f'/spectrometer/status: {self._fb_spectro}'

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_pose_stamped(self, x: float, y: float, z: float = 0.0, frame_id: str = 'map'):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        return ps


# ─────────────────────────────────────────────────────────────────────────────
# OmniLink Master
# ─────────────────────────────────────────────────────────────────────────────

class AnubixOmniLinkMaster:
    """
    Polls OmniLink memory, detects new commands, dispatches to ROS bridge,
    and sends feedback back to agent.

    Clean separation: this class handles OmniLink, AnubixROSBridge handles ROS.
    """

    AGENT_NAME = "ANUBIX"
    ENGINE = "g1-engine"

    # Delegation-hijack detection patterns
    DELEGATION_PATTERNS = (
        re.compile(r'\[Delegation to ["\']supervisor["\']', re.IGNORECASE),
        re.compile(r'Agent ["\']supervisor["\'] not found', re.IGNORECASE),
        re.compile(r'Ensure the agent profile exists', re.IGNORECASE),
    )

    def __init__(self, omni_key: str, bridge: AnubixROSBridge, poll_interval: float = 3.0):
        self.client = OmniLinkClient(omni_key=omni_key, timeout=120)
        self.bridge = bridge
        self.poll_interval = poll_interval
        self._mem_len = 0
        self._last_seen_fingerprint: Optional[str] = None
        self._running = False

    def run(self):
        """Main loop: poll OmniLink, dispatch commands, send feedback."""
        self._running = True

        log.info("=" * 70)
        log.info("  ANUBIX ROS Master Node - OmniLink ↔ ROS 2 Bridge")
        log.info("=" * 70)

        # Stabilize memory cursor to prevent replaying history
        log.info("[INIT] Stabilizing OmniLink memory cursor...")
        self._sync_memory_stable()
        log.info(f"[READY] Memory at {self._mem_len} msgs, fingerprint locked")
        log.info("        Agent ready. Fire tasks via OmniLink web UI.")
        log.info("        Press Ctrl+C to stop.")
        log.info("-" * 70)

        try:
            while self._running:
                try:
                    self._poll()
                except OmniLinkAPIError as e:
                    log.error(f"[API] {e.status_code}: {e.body}")
                except Exception as e:
                    log.error(f"[ERROR] {e}", exc_info=True)
                time.sleep(self.poll_interval)
        except KeyboardInterrupt:
            log.info("\n[STOP] Ctrl+C received")
        finally:
            self._running = False
            log.info("[SHUTDOWN] Master node stopped")

    # ── Memory Polling ────────────────────────────────────────────────────────

    @staticmethod
    def _fingerprint(msg: dict) -> str:
        """Unique identifier for a memory message."""
        role = msg.get('role', '')
        text = ''.join(p.get('text', '') for p in msg.get('parts', []))
        return f'{role}::{text}'

    def _sync_memory_stable(self, max_attempts: int = 8, delay: float = 0.5):
        """Stabilize memory cursor to prevent replaying history on startup."""
        prev_len = -1
        prev_fp = None
        for attempt in range(max_attempts):
            try:
                memory = self.client.get_memory(self.AGENT_NAME) or []
            except Exception as e:
                log.error(f"[MEM] get_memory failed (attempt {attempt+1}): {e}")
                time.sleep(delay)
                continue

            cur_len = len(memory)
            cur_fp = self._fingerprint(memory[-1]) if memory else None

            if cur_len == prev_len and cur_fp == prev_fp:
                self._mem_len = cur_len
                self._last_seen_fingerprint = cur_fp
                return

            prev_len, prev_fp = cur_len, cur_fp
            time.sleep(delay)

        # Couldn't stabilize, use what we have
        self._mem_len = prev_len if prev_len >= 0 else 0
        self._last_seen_fingerprint = prev_fp
        log.warning(f"[MEM] Cursor not fully stable; locked at {self._mem_len} msgs")

    def _sync_memory(self):
        """Lightweight cursor refresh after processing."""
        try:
            memory = self.client.get_memory(self.AGENT_NAME) or []
            self._mem_len = len(memory)
            if memory:
                self._last_seen_fingerprint = self._fingerprint(memory[-1])
        except Exception as e:
            log.error(f"[MEM] sync_memory failed: {e}")

    def _poll(self):
        """Check for new messages in OmniLink memory."""
        log.debug("[POLL] Polling OmniLink memory...")
        try:
            memory = self.client.get_memory(self.AGENT_NAME)
        except Exception as e:
            log.error(f"[POLL] get_memory failed: {e}")
            return

        if not memory:
            log.debug("[POLL] Memory is empty")
            return

        log.debug(f"[POLL] Memory has {len(memory)} messages, cursor at {self._mem_len}")

        # Find start point using fingerprint
        if self._last_seen_fingerprint is not None:
            anchor = -1
            for i in range(len(memory) - 1, -1, -1):
                if self._fingerprint(memory[i]) == self._last_seen_fingerprint:
                    anchor = i
                    break

            if anchor < 0:
                # Fingerprint not found - memory might have been cleared or reset
                # Use _mem_len as fallback instead of skipping
                log.warning("[POLL] Fingerprint not found - using mem_len as fallback")
                log.warning(f"       Last seen: {self._last_seen_fingerprint[:50]}...")
                log.warning(f"       Memory length: {len(memory)}, cursor: {self._mem_len}")
                start = self._mem_len
            else:
                start = anchor + 1
        else:
            start = self._mem_len

        if start >= len(memory):
            log.debug(f"[POLL] No new messages (start={start}, len={len(memory)})")
            return

        new_messages = memory[start:]
        self._mem_len = len(memory)
        self._last_seen_fingerprint = self._fingerprint(memory[-1])

        log.info(f"[POLL] {len(new_messages)} new message(s) detected")

        for msg in new_messages:
            role = msg.get('role', '')
            if role != 'model':
                log.debug(f"[POLL] Skipping non-model message (role={role})")
                continue

            # Extract text and tool calls
            text = ''.join(p.get('text', '') for p in msg.get('parts', []))
            tool_calls = []
            for part in msg.get('parts', []):
                if 'tool_use' in part:
                    tool_calls.append(part['tool_use'])

            log.info(f"[POLL] Model message: {len(text)} chars text, {len(tool_calls)} tool calls")

            # Try tool calls first (preferred method)
            if tool_calls:
                log.info(f"[POLL] Processing {len(tool_calls)} tool call(s)")
                cmds = self._parse_tool_calls(tool_calls)
            else:
                # Fallback to text parsing (old method)
                log.debug(f"[POLL] No tool calls, parsing text...")
                cmds = parse_commands(text)

            log.info(f"[POLL] Parsed {len(cmds)} command(s)")

            # Delegation hijack recovery (only for text-based)
            if not cmds and not tool_calls and self._is_delegation_hijack(text):
                log.warning("[POLL] Delegation hijack detected - recovering...")
                recovered = self._recover_from_delegation()
                if recovered:
                    text = recovered
                    cmds = parse_commands(text)

            if not cmds:
                log.warning("[POLL] No commands found!")
                if text:
                    log.warning(f"[POLL] Agent text: {text[:300]}")
                continue

            if text and tool_calls:
                self._print_agent(f"TOOL CALLS: {len(tool_calls)}")
            elif text:
                self._print_agent(text)

            # NEW ARCHITECTURE: Execute commands sequentially
            if len(cmds) > 1:
                log.warning(f"[POLL] Agent emitted {len(cmds)} commands - expected 1 per response!")

            feedback = self._dispatch(cmds)
            if feedback:
                self._execution_loop(feedback)

            self._sync_memory()
            return  # Process one batch per poll tick

    # ── Tool Call Parsing ─────────────────────────────────────────────────────

    def _parse_tool_calls(self, tool_calls: list) -> list:
        """
        Parse tool calls from agent response and convert to command tuples.

        Tool call format from OmniLink:
        {
            'name': 'supervisor_nav_goal',
            'input': {'x': 40.0, 'y': 45.0}
        }

        Returns: List of (cmd_type, FakeMatch) tuples compatible with _execute_one()
        """
        import re

        class FakeMatch:
            """Fake regex match object to work with existing _execute_one() code."""
            def __init__(self, groups):
                self.groups_list = groups
                self.lastindex = len(groups) - 1 if groups else 0

            def group(self, idx):
                if idx == 0:
                    return f"<tool_call:{self.groups_list}>"
                if idx <= len(self.groups_list):
                    return self.groups_list[idx - 1]
                return ''

        results = []

        for tc in tool_calls:
            name = tc.get('name', '')
            input_params = tc.get('input', {})

            log.info(f"[TOOL] {name}({input_params})")

            # Map tool names to command types
            if name == 'supervisor_robot_id':
                results.append(('robot_id', FakeMatch([input_params.get('robot_id', '')])))

            elif name == 'supervisor_task_id':
                results.append(('task_id', FakeMatch([input_params.get('task_id', '')])))

            elif name == 'supervisor_nav_vision':
                vision_str = 'true' if input_params.get('vision') else 'false'
                results.append(('nav_vision', FakeMatch([vision_str])))

            elif name == 'supervisor_nav_goal':
                x = str(input_params.get('x', 0))
                y = str(input_params.get('y', 0))
                results.append(('nav_goal', FakeMatch([x, y])))

            elif name == 'supervisor_nav_goal_home':
                results.append(('nav_goal_home', FakeMatch([])))

            elif name == 'supervisor_target_camera':
                cam = str(input_params.get('camera_number', 1))
                results.append(('target_camera', FakeMatch([cam])))

            elif name == 'supervisor_perception_goal':
                task = input_params.get('task_type', 'disease').lower()
                results.append(('perception_goal', FakeMatch([task])))

            elif name == 'supervisor_arm_nav_goal':
                signal = input_params.get('signal', 'move').lower()
                results.append(('arm_nav_goal', FakeMatch([signal])))

            elif name == 'supervisor_grip':
                action_str = 'true' if input_params.get('action') else 'false'
                results.append(('grip', FakeMatch([action_str])))

            elif name == 'supervisor_spectral_target':
                task = input_params.get('task_type', 'disease').lower()
                rid = input_params.get('robot_id', '')
                tid = input_params.get('task_id', '')
                results.append(('spectral_target', FakeMatch([task, rid, tid])))

            elif name == 'supervisor_force_stop':
                results.append(('force_stop', FakeMatch([])))

            else:
                log.warning(f"[TOOL] Unknown tool: {name}")

        return results

    # ── Delegation Hijack Recovery ────────────────────────────────────────────

    def _is_delegation_hijack(self, text: str) -> bool:
        return any(p.search(text) for p in self.DELEGATION_PATTERNS)

    def _recover_from_delegation(self, max_attempts: int = 2) -> Optional[str]:
        """Re-prompt agent to emit plain-text commands."""
        prompts = [
            "SYSTEM NOTE: your previous response was intercepted by the delegation "
            "pipeline because it parsed `supervisor/` as an agent name. There is no "
            "agent called 'supervisor' – it is a literal command prefix. Re-emit the "
            "same command as PLAIN TEXT, exactly as written in your custom instructions. "
            "Do not delegate. Do not call tools.",
            "Please respond with the supervisor/* command for the current step as "
            "plain text. No tool calls, no delegation.",
        ]

        for i in range(max_attempts):
            prompt = prompts[min(i, len(prompts) - 1)]
            try:
                resp = self.client.chat(
                    prompt=prompt,
                    agent_name=self.AGENT_NAME,
                    engine=self.ENGINE,
                )
            except OmniLinkAPIError as e:
                log.error(f"[RECOVER] {e.status_code}: {e.body}")
                return None

            text = resp.get('text', '') or ''
            if self._is_delegation_hijack(text):
                log.warning(f"[RECOVER] Attempt {i+1} still hijacked")
                continue

            if parse_commands(text):
                log.info(f"[RECOVER] Attempt {i+1} returned valid commands")
                return text

        log.error("[RECOVER] All attempts failed")
        return None

    # ── Execution Loop ────────────────────────────────────────────────────────

    def _execution_loop(self, initial_feedback: str):
        """
        Send feedback to agent, wait for next command, execute it, repeat.
        This loop implements the sequential one-command-at-a-time architecture.
        """
        feedback = initial_feedback
        loop_iter = 0

        while feedback and self._running:
            loop_iter += 1
            log.info(f"\n{'─'*60}")
            log.info(f"[FEEDBACK → AGENT] (iteration {loop_iter})")
            log.info(f"{feedback}")
            log.info(f"{'─'*60}")

            try:
                resp = self.client.chat(
                    prompt=feedback,
                    agent_name=self.AGENT_NAME,
                    engine=self.ENGINE,
                )
            except OmniLinkAPIError as e:
                log.error(f"[CHAT] {e.status_code}: {e.body}")
                break
            except Exception as e:
                log.error(f"[CHAT] {e}", exc_info=True)
                break

            agent_text = resp.get('text', '')

            # Delegation hijack recovery
            if not parse_commands(agent_text) and self._is_delegation_hijack(agent_text):
                log.warning("[LOOP] Mid-loop delegation hijack - recovering...")
                recovered = self._recover_from_delegation()
                if recovered:
                    agent_text = recovered

            self._print_agent(agent_text)

            # Force stop check
            if re.search(r'supervisor/force_stop', agent_text, re.IGNORECASE):
                log.warning("[EMERGENCY] force_stop detected!")
                self.bridge.execute('force_stop')
                break

            cmds = parse_commands(agent_text)
            if not cmds:
                log.info("[DONE] No more commands - mission complete")
                break

            if len(cmds) > 1:
                log.warning(f"[LOOP] Agent emitted {len(cmds)} commands - expected 1!")

            feedback = self._dispatch(cmds)
            if not feedback:
                log.warning("[LOOP] No feedback - yielding to poll")
                break

    # ── Dispatch ──────────────────────────────────────────────────────────────

    def _dispatch(self, cmds: list) -> str:
        """
        Execute commands in PARSE ORDER (not priority order).
        Sequential architecture: agent emits ONE command, gets confirmation, emits next.
        """
        feedbacks = []

        for cmd_type, match in cmds:
            log.info(f"[CMD] ► {match.group(0)}")
            fb = self._execute_one(cmd_type, match)
            if fb:
                log.info(f"[FB]  ◄ {fb.replace(chr(10), ' | ')}")
                feedbacks.append(fb)
            else:
                log.warning(f"[FB]  ◄ (no feedback)")

            if cmd_type == 'force_stop':
                break

        return '\n'.join(feedbacks)

    def _execute_one(self, cmd_type: str, match: re.Match) -> Optional[str]:
        """Execute a single command via the ROS bridge."""
        if cmd_type == 'force_stop':
            return self.bridge.execute('force_stop')

        if cmd_type == 'robot_id':
            return self.bridge.execute('robot_id', robot_id=match.group(1))

        if cmd_type == 'task_id':
            return self.bridge.execute('task_id', task_id=match.group(1))

        if cmd_type == 'nav_vision':
            vision = match.group(1).lower() == 'true'
            return self.bridge.execute('nav_vision', vision=vision)

        if cmd_type == 'nav_goal':
            return self.bridge.execute('nav_goal',
                                      x=float(match.group(1)),
                                      y=float(match.group(2)))

        if cmd_type == 'nav_goal_home':
            return self.bridge.execute('nav_goal_home')

        if cmd_type == 'target_camera':
            return self.bridge.execute('target_camera',
                                      camera_number=int(match.group(1)))

        if cmd_type == 'perception_goal':
            return self.bridge.execute('perception_goal',
                                      task_type=match.group(1).lower())

        if cmd_type == 'arm_nav_goal':
            return self.bridge.execute('arm_nav_goal',
                                      signal=match.group(1).lower())

        if cmd_type == 'grip':
            action = match.group(1).lower() == 'true'
            return self.bridge.execute('grip', action=action)

        if cmd_type == 'spectral_target':
            robot_id = (match.group(2) or '') if match.lastindex and match.lastindex >= 2 else ''
            task_id = (match.group(3) or '') if match.lastindex and match.lastindex >= 3 else ''
            return self.bridge.execute('spectral_target',
                                      task_type=match.group(1).lower(),
                                      robot_id=robot_id,
                                      task_id=task_id)

        log.error(f"[DISPATCH] Unknown command type: {cmd_type}")
        return None

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _print_agent(text: str):
        log.info(f"\n{'═'*60}")
        log.info("  ANUBIX AGENT")
        log.info(f"{'═'*60}")
        for line in text.splitlines():
            log.info(f"  {line}")
        log.info("═" * 60)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="ANUBIX ROS Master Node")
    p.add_argument('--poll', type=float, default=3.0,
                   help="OmniLink poll interval (default 3.0s)")
    p.add_argument('--feedback-timeout', type=float, default=120.0,
                   help="ROS feedback timeout (default 120s)")
    p.add_argument('--arm-home', type=float, nargs=3, default=[0.0, 0.0, 0.3],
                   metavar=('X', 'Y', 'Z'), help="Arm home pose")
    p.add_argument('--robot-id', type=str, default='',
                   help="Default robot ID (can be overridden by commands)")
    p.add_argument('--task-id', type=str, default='',
                   help="Default task ID (can be overridden by commands)")
    p.add_argument('--verbose', action='store_true',
                   help="Enable debug logging")
    return p.parse_args()


def main():
    print("=" * 70)
    print("  ANUBIX ROS MASTER NODE V3 - STARTING")
    print("=" * 70)

    args = parse_args()
    print(f"Arguments parsed: poll={args.poll}, feedback_timeout={args.feedback_timeout}")

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        print("Verbose logging enabled")

    omni_key = os.environ.get('OMNI_KEY', '').strip()
    if not omni_key:
        print("ERROR: OMNI_KEY not set. Export OMNI_KEY=olink_YOUR_KEY")
        sys.exit(1)

    print(f"OMNI_KEY found: {omni_key[:15]}...")
    print("Initializing ROS 2...")

    # Initialize ROS 2
    rclpy.init()
    print("ROS 2 initialized")

    # Create ROS bridge node
    print("Creating ROS bridge node...")
    bridge = AnubixROSBridge(
        feedback_timeout=args.feedback_timeout,
        arm_home_pose=tuple(args.arm_home),
        robot_id=args.robot_id,
        task_id=args.task_id,
    )
    print("ROS bridge node created")

    # Start executor in background thread
    print("Starting ROS executor thread...")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)

    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()
    print("ROS executor thread started")

    # Allow QoS handshakes to settle
    print("Waiting for QoS handshakes...")
    time.sleep(0.5)
    print("QoS handshakes complete")

    try:
        # Run OmniLink master
        print("Creating OmniLink master...")
        master = AnubixOmniLinkMaster(
            omni_key=omni_key,
            bridge=bridge,
            poll_interval=args.poll,
        )
        print("Starting OmniLink master run loop...")
        master.run()
    finally:
        log.info("[SHUTDOWN] Cleaning up...")
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        log.info("[SHUTDOWN] Complete")


if __name__ == '__main__':
    main()
