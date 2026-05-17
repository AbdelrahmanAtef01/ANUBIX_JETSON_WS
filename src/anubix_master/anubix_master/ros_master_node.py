#!/usr/bin/env python3
"""
ANUBIX ROS 2 Master Node - Tool Callback Architecture
======================================================
Bridges ANUBIX OmniLink agent to four ROS 2 subsystems on Jetson + RPi.

Architecture (toolCallbackUrl):
- Runs an HTTP server on a configurable port (default 5055)
- Registers the server URL as toolCallbackUrl in the ANUBIX agent profile
- When user sends a message via OmniLink web UI:
  1. OmniLink platform calls AI engine with tool definitions
  2. AI responds with structured toolCalls (e.g. supervisor_nav_goal)
  3. Web UI frontend POSTs tool call to our HTTP server
  4. We execute the command via ROS 2, wait for feedback
  5. Return the result JSON to the frontend
  6. Frontend sends result back to AI for the next tool call
  7. Loop continues until no more tool calls (maxToolRounds)

This is the correct OmniLink Agents architecture - no memory polling needed.
The web UI drives the tool-call loop automatically.
"""

import os
import sys
import json
import time
import logging
import argparse
import threading
import http.server
import socket
import subprocess
import concurrent.futures
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
# ROS 2 Bridge
# ─────────────────────────────────────────────────────────────────────────────

class AnubixROSBridge(Node):
    """
    ROS 2 node that publishes supervisor commands and waits for feedback.

    Each execute() call:
    1. Publishes the command to the appropriate supervisor/* topic
    2. Blocks until feedback arrives on the corresponding status topic
    3. Returns formatted feedback string
    """

    def __init__(self,
                 feedback_timeout: float = 120.0,
                 arm_home_pose: Tuple[float, float, float] = (0.0, 0.0, 0.3),
                 robot_id: str = '',
                 task_id: str = ''):
        super().__init__('anubix_master')

        self.feedback_timeout = feedback_timeout
        self.arm_home_pose = arm_home_pose

        self._context_robot_id = robot_id
        self._context_task_id = task_id

        self._force_stopped = False
        self._latest_target_pose: Optional[Pose] = None
        self._target_pose_lock = threading.Lock()
        self._post_release_retract = False

        self._sub_group = ReentrantCallbackGroup()

        # QoS profiles
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        trigger_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            durability=DurabilityPolicy.VOLATILE,
        )
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
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

    # ── Public API ────────────────────────────────────────────────────────────

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
        time.sleep(0.05)
        return f'/context/robot_id: {robot_id}'

    def _do_task_id(self, task_id: str) -> str:
        self._context_task_id = task_id
        self.pub_task_id.publish(String(data=task_id))
        log.info(f"[TX] /supervisor/task_id = '{task_id}'")
        time.sleep(0.05)
        return f'/context/task_id: {task_id}'

    def _do_nav_vision(self, vision: bool) -> str:
        self.pub_nav_vision.publish(Bool(data=vision))
        log.info(f"[TX] /supervisor/nav_vision = {vision}")
        time.sleep(0.1)
        return f'/supervisor/nav_vision: {vision}'

    def _do_nav_goal(self, x: float, y: float) -> str:
        ps = self._make_pose_stamped(x, y, 0.0)
        self._ev_nav.clear()
        self._fb_nav = None

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
        time.sleep(0.05)
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
            if self._fb_gripper == 'successful_release':
                self._post_release_retract = True
                log.info("[GRIP] Post-release retract armed")
            return gripper_line

        if not self._ev_touch.wait(self.feedback_timeout):
            log.warning("[TIMEOUT] No /arm/touch_status received, assuming false")
            touch_line = '/arm/touch_status: false'
        else:
            touch_line = f'/arm/touch_status: {"true" if self._fb_touch else "false"}'

        return f'{gripper_line}\n{touch_line}'

    def _do_spectral_target(self, task_type: str, robot_id: str = '', task_id: str = '') -> str:
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
# Tool Callback HTTP Server
# ─────────────────────────────────────────────────────────────────────────────

class ToolCallbackHandler(http.server.BaseHTTPRequestHandler):
    """HTTP handler for OmniLink tool callbacks.

    The OmniLink web UI frontend POSTs tool calls here when the AI agent
    emits a structured toolCall. We execute the tool via ROS 2 and return
    the result so the frontend can feed it back to the AI.

    POST body format from OmniLink frontend:
        {"tool": "supervisor_nav_goal", "x": 40.0, "y": 45.0}

    Response format:
        {"status": "ok", "tool": "supervisor_nav_goal", "result": {...}}
    """

    bridge: Optional['AnubixROSBridge'] = None

    def log_message(self, fmt, *args):
        log.debug(f"[HTTP] {fmt % args}")

    def _cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Private-Network", "true")

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors_headers()
        self.send_header("Access-Control-Max-Age", "86400")
        self.end_headers()

    def do_GET(self):
        if self.path == "/health":
            body = json.dumps({"status": "ok", "node": "anubix_master"}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors_headers()
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/tool":
            self.send_error(404)
            return

        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length).decode("utf-8", errors="replace")
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            data = {}

        tool_name = str(data.get("tool", "")).strip()
        tool_kwargs = {k: v for k, v in data.items() if k != "tool"}

        log.info(f"[TOOL CALLBACK] {tool_name}({tool_kwargs})")

        result = self._execute_tool(tool_name, tool_kwargs)

        log.info(f"[TOOL RESULT] {tool_name} -> {result}")

        body = json.dumps({"status": "ok", "tool": tool_name, "result": result}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self._cors_headers()
        self.end_headers()
        self.wfile.write(body)

    def _execute_tool(self, tool_name: str, kwargs: dict) -> dict:
        """Map OmniLink tool call to ROS bridge execution."""
        bridge = self.__class__.bridge
        if bridge is None:
            return {"error": "ROS bridge not initialized"}

        try:
            if tool_name == "supervisor_robot_id":
                fb = bridge.execute('robot_id', robot_id=str(kwargs.get('robot_id', '')))
                return {"feedback": fb, "message": f"Robot ID set to {kwargs.get('robot_id')}"}

            elif tool_name == "supervisor_task_id":
                fb = bridge.execute('task_id', task_id=str(kwargs.get('task_id', '')))
                return {"feedback": fb, "message": f"Task ID set to {kwargs.get('task_id')}"}

            elif tool_name == "supervisor_nav_vision":
                vision = bool(kwargs.get('vision', False))
                fb = bridge.execute('nav_vision', vision=vision)
                return {"feedback": fb, "message": f"Nav vision set to {vision}"}

            elif tool_name == "supervisor_nav_goal":
                x = float(kwargs.get('x', 0))
                y = float(kwargs.get('y', 0))
                fb = bridge.execute('nav_goal', x=x, y=y)
                return {"feedback": fb, "message": f"Navigation to ({x}, {y}) complete"}

            elif tool_name == "supervisor_nav_goal_home":
                fb = bridge.execute('nav_goal_home')
                return {"feedback": fb, "message": "Navigation to home complete"}

            elif tool_name == "supervisor_target_camera":
                cam = int(kwargs.get('camera_number', 1))
                fb = bridge.execute('target_camera', camera_number=cam)
                return {"feedback": fb, "message": f"Camera {cam} selected"}

            elif tool_name == "supervisor_perception_goal":
                task_type = str(kwargs.get('task_type', 'disease')).lower()
                fb = bridge.execute('perception_goal', task_type=task_type)
                return {"feedback": fb, "message": f"Perception ({task_type}) complete"}

            elif tool_name == "supervisor_arm_nav_goal":
                signal = str(kwargs.get('signal', 'move')).lower()
                fb = bridge.execute('arm_nav_goal', signal=signal)
                return {"feedback": fb, "message": f"Arm nav ({signal}) complete"}

            elif tool_name == "supervisor_grip":
                action = bool(kwargs.get('action', False))
                fb = bridge.execute('grip', action=action)
                return {"feedback": fb, "message": f"Gripper {'close' if action else 'open'} complete"}

            elif tool_name == "supervisor_spectral_target":
                task_type = str(kwargs.get('task_type', 'disease')).lower()
                robot_id = str(kwargs.get('robot_id', ''))
                task_id = str(kwargs.get('task_id', ''))
                fb = bridge.execute('spectral_target',
                                    task_type=task_type,
                                    robot_id=robot_id,
                                    task_id=task_id)
                return {"feedback": fb, "message": f"Spectral scan ({task_type}) complete"}

            elif tool_name == "supervisor_force_stop":
                fb = bridge.execute('force_stop')
                return {"feedback": fb, "message": "Emergency stop executed"}

            else:
                log.warning(f"[TOOL] Unknown tool: {tool_name}")
                return {"error": f"Unknown tool: {tool_name}"}

        except Exception as e:
            log.error(f"[TOOL] Exception executing {tool_name}: {e}")
            return {"error": str(e)}


# ─────────────────────────────────────────────────────────────────────────────
# Profile Management
# ─────────────────────────────────────────────────────────────────────────────

SUPERVISOR_TOOLS = [
    {
        "name": "supervisor_robot_id",
        "description": "Set the robot context ID for tracking",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {
                    "type": "string",
                    "description": "UUID or identifier for the robot instance"
                }
            },
            "required": ["robot_id"]
        }
    },
    {
        "name": "supervisor_task_id",
        "description": "Set the task context ID for tracking",
        "parameters": {
            "type": "object",
            "properties": {
                "task_id": {
                    "type": "string",
                    "description": "UUID or identifier for the specific task"
                }
            },
            "required": ["task_id"]
        }
    },
    {
        "name": "supervisor_nav_vision",
        "description": "Set navigation vision mode (true=stop 1m before target for camera, false=drive all the way)",
        "parameters": {
            "type": "object",
            "properties": {
                "vision": {
                    "type": "boolean",
                    "description": "Enable vision mode (stop before target)"
                }
            },
            "required": ["vision"]
        }
    },
    {
        "name": "supervisor_nav_goal",
        "description": "Navigate robot to specific coordinates. Returns /nav/status: point_reached|blocked|failure",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "X coordinate in meters"},
                "y": {"type": "number", "description": "Y coordinate in meters"}
            },
            "required": ["x", "y"]
        }
    },
    {
        "name": "supervisor_nav_goal_home",
        "description": "Return robot to home position (0, 0). Returns /nav/status",
        "parameters": {"type": "object", "properties": {}}
    },
    {
        "name": "supervisor_target_camera",
        "description": "Select camera for perception (1=wide-angle, 2=telephoto)",
        "parameters": {
            "type": "object",
            "properties": {
                "camera_number": {
                    "type": "string",
                    "description": "Camera number: '1'=wide-angle, '2'=telephoto",
                    "enum": ["1", "2"]
                }
            },
            "required": ["camera_number"]
        }
    },
    {
        "name": "supervisor_perception_goal",
        "description": "Start perception task. Returns /perception/status: found|not_found",
        "parameters": {
            "type": "object",
            "properties": {
                "task_type": {
                    "type": "string",
                    "description": "Type of perception task",
                    "enum": ["disease", "water_stress", "harvest_status"]
                }
            },
            "required": ["task_type"]
        }
    },
    {
        "name": "supervisor_arm_nav_goal",
        "description": "Move robotic arm to target or home position. Returns /arm/arm_status: success|block|mechanical_error",
        "parameters": {
            "type": "object",
            "properties": {
                "signal": {
                    "type": "string",
                    "description": "move=go to target pose, home=return to safe position",
                    "enum": ["move", "home"]
                }
            },
            "required": ["signal"]
        }
    },
    {
        "name": "supervisor_grip",
        "description": "Control gripper. Returns /arm/gripper_status and /arm/touch_status",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "boolean",
                    "description": "true=close gripper, false=open gripper"
                }
            },
            "required": ["action"]
        }
    },
    {
        "name": "supervisor_spectral_target",
        "description": "Run spectrometer scan on target. Returns /spectrometer/status: success|failure",
        "parameters": {
            "type": "object",
            "properties": {
                "task_type": {
                    "type": "string",
                    "description": "Type of spectral analysis",
                    "enum": ["disease", "water_stress", "harvest_status"]
                },
                "robot_id": {
                    "type": "string",
                    "description": "Robot ID (optional, uses context if not provided)"
                },
                "task_id": {
                    "type": "string",
                    "description": "Task ID (optional, uses context if not provided)"
                }
            },
            "required": ["task_type"]
        }
    },
    {
        "name": "supervisor_force_stop",
        "description": "Emergency stop - halts all robot operations immediately",
        "parameters": {"type": "object", "properties": {}}
    },
]


def ensure_profile(client: OmniLinkClient, tool_callback_url: str, agent_prompt: str):
    """Create or update the ANUBIX agent profile with toolCallbackUrl."""
    settings = {
        "agentName": "ANUBIX",
        "mainTask": agent_prompt,
        "agentPersonality": "Professional",
        "allowToolUse": True,
        "availableTools": ",".join(t["name"] for t in SUPERVISOR_TOOLS),
        "availableToolDetails": SUPERVISOR_TOOLS,
        "toolCallbackUrl": tool_callback_url,
        "maxToolRounds": 15,
    }

    try:
        profiles = client.list_profiles()
        existing = next((p for p in profiles if p.get('name') == 'ANUBIX'), None)

        if existing:
            profile_id = existing['id']
            client.update_profile(profile_id, settings=settings)
            log.info(f"[PROFILE] Updated ANUBIX profile (ID: {profile_id})")
            log.info(f"[PROFILE] toolCallbackUrl = {tool_callback_url}")
        else:
            result = client.create_profile(name="ANUBIX", settings=settings)
            profile_id = result['id']
            log.info(f"[PROFILE] Created ANUBIX profile (ID: {profile_id})")
            log.info(f"[PROFILE] toolCallbackUrl = {tool_callback_url}")

    except Exception as e:
        log.error(f"[PROFILE] Failed to set profile: {e}")
        log.error("[PROFILE] Tool callbacks will not work without a valid profile!")
        raise


def load_agent_prompt() -> str:
    """Load agent prompt from file, or use embedded default."""
    search_paths = [
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'agent_config', 'ANUBIX_AGENT_PROMPT_v3_TOOLCALLS.txt'),
        os.path.join(os.path.dirname(__file__), '..', 'agent_config', 'ANUBIX_AGENT_PROMPT_v3_TOOLCALLS.txt'),
        'ANUBIX_AGENT_PROMPT_v3_TOOLCALLS.txt',
    ]

    for path in search_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            with open(abs_path, 'r', encoding='utf-8') as f:
                content = f.read()
            log.info(f"[PROMPT] Loaded from {abs_path} ({len(content)} bytes)")
            return content

    log.warning("[PROMPT] No prompt file found, using embedded default")
    return (
        "You are Anubix, an autonomous agritech robot executing precision agricultural tasks.\n"
        "Your primary duties: measure water stress, detect diseases, check harvest status.\n\n"
        "CRITICAL: Emit ONE tool call per response. Wait for confirmation before the next.\n"
        "Follow the 11-step execution sequence exactly. Do not skip steps.\n"
        "Do not add explanatory text - only use tool calls.\n"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="ANUBIX ROS Master Node (Tool Callback Architecture)")
    p.add_argument('--port', type=int, default=5055,
                   help="Tool callback HTTP server port (default 5055)")
    p.add_argument('--host', type=str, default='0.0.0.0',
                   help="Tool callback HTTP server bind address (default 0.0.0.0)")
    p.add_argument('--callback-url', type=str, default='',
                   help="Override toolCallbackUrl (default: http://127.0.0.1:<port>/tool)")
    p.add_argument('--feedback-timeout', type=float, default=120.0,
                   help="ROS feedback timeout (default 120s)")
    p.add_argument('--arm-home', type=float, nargs=3, default=[0.0, 0.0, 0.3],
                   metavar=('X', 'Y', 'Z'), help="Arm home pose")
    p.add_argument('--robot-id', type=str, default='',
                   help="Default robot ID")
    p.add_argument('--task-id', type=str, default='',
                   help="Default task ID")
    p.add_argument('--verbose', action='store_true',
                   help="Enable debug logging")
    args, unknown = p.parse_known_args()
    if unknown:
        log.debug(f"Ignoring unknown arguments (ROS 2 remappings): {unknown}")
    return args


def _get_wifi_ip() -> str:
    """Return the IP of the interface that has internet access (WiFi on Jetson)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def _start_ngrok(port: int) -> str:
    """Start ngrok tunnel for the given port and return the public HTTPS URL."""
    # Kill any existing ngrok process to avoid conflicts
    subprocess.run(["pkill", "-f", "ngrok"], capture_output=True)
    time.sleep(1)

    proc = subprocess.Popen(
        ["ngrok", "http", str(port), "--log=stdout", "--log-format=json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    # Read ngrok JSON log lines until we see the tunnel URL (up to 15s)
    deadline = time.time() + 15
    while time.time() < deadline:
        line = proc.stdout.readline()
        if not line:
            break
        try:
            entry = json.loads(line.decode())
            # ngrok emits {"url":"https://..."} when the tunnel is ready
            url = entry.get("url", "")
            if url.startswith("https://"):
                print(f"  ngrok tunnel established: {url}")
                return url.rstrip("/")
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass

    proc.kill()
    raise RuntimeError("ngrok did not start within 15 seconds — is it installed and authenticated?")


def main():
    print("=" * 70)
    print("  ANUBIX ROS MASTER NODE - Tool Callback Architecture")
    print("=" * 70)

    args = parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    omni_key = os.environ.get('OMNI_KEY', '').strip()
    if not omni_key:
        print("ERROR: OMNI_KEY not set. Export OMNI_KEY=olink_YOUR_KEY")
        sys.exit(1)

    print(f"  OMNI_KEY: {omni_key[:15]}...")
    print(f"  Port: {args.port}")
    print(f"  Host: {args.host}")

    # Determine tool callback URL
    # OmniLink's CSP only allows localhost/* — use SSH port forwarding from the
    # remote device: ssh -L 5055:localhost:5055 user@<jetson-ip>
    tool_callback_url = args.callback_url or f"http://localhost:{args.port}/tool"
    print(f"  toolCallbackUrl: {tool_callback_url}")

    # Initialize ROS 2
    print("  Initializing ROS 2...")
    rclpy.init()

    bridge = AnubixROSBridge(
        feedback_timeout=args.feedback_timeout,
        arm_home_pose=tuple(args.arm_home),
        robot_id=args.robot_id,
        task_id=args.task_id,
    )

    # Start ROS executor in background thread
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()
    time.sleep(0.5)
    print("  ROS 2 initialized and spinning")

    # Set the bridge reference on the handler class
    ToolCallbackHandler.bridge = bridge

    # Start HTTP tool callback server
    server = http.server.ThreadingHTTPServer(
        (args.host, args.port), ToolCallbackHandler
    )
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"  Tool callback server listening on {args.host}:{args.port}")

    # Update agent profile with toolCallbackUrl
    print("  Updating ANUBIX agent profile...")
    client = OmniLinkClient(omni_key=omni_key, timeout=30)
    agent_prompt = load_agent_prompt()

    try:
        ensure_profile(client, tool_callback_url, agent_prompt)
    except Exception as e:
        log.error(f"Failed to configure profile: {e}")
        log.error("Continuing anyway - profile may need manual configuration")

    print()
    print("=" * 70)
    print("  ANUBIX READY")
    print("=" * 70)
    print()
    print(f"  Tool callback URL: {tool_callback_url}")
    print(f"  How it works:")
    print(f"    1. Open OmniLink web UI: https://www.omnilink-agents.com")
    print(f"    2. Select 'ANUBIX' agent")
    print(f"    3. Send a task (e.g. 'Check disease at 40,45 with robot_id=abc task_id=def')")
    print(f"    4. The AI will emit tool calls -> web UI POSTs them here -> we execute via ROS")
    print(f"    5. Results flow back to AI automatically")
    print()
    print(f"  Press Ctrl+C to stop.")
    print("-" * 70)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C received")
    finally:
        log.info("[SHUTDOWN] Cleaning up...")
        server.shutdown()
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        log.info("[SHUTDOWN] Complete")


if __name__ == '__main__':
    main()
