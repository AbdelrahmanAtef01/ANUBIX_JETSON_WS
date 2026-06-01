#!/usr/bin/env python3
"""
anubix_gripper — gripper_node.py
=================================
ROS 2 node for the myGripperF100 (Elephant Robotics) on the Anubix system.

Bridges the supervisor interface used by the master node with the actual
gripper hardware over USB-RS485 (Modbus RTU).

SUPERVISOR INTERFACE (master node compatibility):
  Subscribe : /supervisor/grip        (std_msgs/Bool)   — True=pick, False=release
  Subscribe : /supervisor/force_stop  (std_msgs/Bool)   — emergency stop
  Publish   : /arm/gripper_status     (std_msgs/String) — successful_grip|successful_release|mechanical_error
  Publish   : /arm/touch_status       (std_msgs/Bool)   — True when leaf held

DIRECT INTERFACE (for manual testing / gripper_sender):
  Subscribe : /gripper/command        (std_msgs/String)  — pick|open|release|close|stop|status
  Publish   : /gripper/status         (std_msgs/String)  — status feedback
  Publish   : /gripper/position       (std_msgs/Float32) — current position 0-100
  Service   : /gripper/pick           (std_srvs/Trigger)
  Service   : /gripper/open           (std_srvs/Trigger)
  Service   : /gripper/close          (std_srvs/Trigger)
  Service   : /gripper/release        (std_srvs/Trigger)

Connection: USB-to-RS485 adapter -> myGripperF100
  Default port: /dev/ttyACM0 (auto-detect scans /dev/ttyACM* and /dev/ttyUSB*)
  Baud: 115200 | Gripper ID: 14
"""

import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger

from anubix_gripper.elegripper import Gripper


# ═══════════════════════════════════════════════════════════════════════════════
#  QoS PROFILES (match arm node / master node)
# ═══════════════════════════════════════════════════════════════════════════════

_QOS_RELIABLE_VOL = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_QOS_STATUS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


# ═══════════════════════════════════════════════════════════════════════════════
#  HARDWARE STATUS CODES
# ═══════════════════════════════════════════════════════════════════════════════

GRIPPER_HW_STATUS = {
    0: "MOVING",
    1: "STOPPED_EMPTY",
    2: "HOLDING",
    3: "SLIPPED",
}


class PickResult:
    SUCCESS = "SUCCESS"
    FAILED  = "FAILED"
    EMPTY   = "EMPTY"
    TIMEOUT = "TIMEOUT"


# ═══════════════════════════════════════════════════════════════════════════════
#  USB PORT AUTO-DETECTION
# ═══════════════════════════════════════════════════════════════════════════════

def find_gripper_port(preferred_port, baud, gripper_id, logger):
    """
    Try to find a working gripper serial port.
    1. If preferred_port is set and exists, try it first.
    2. Otherwise scan common USB serial ports.
    Returns (Gripper, port_name) or raises RuntimeError.
    """
    import glob as globmod

    candidates = []
    if preferred_port and preferred_port != 'auto':
        candidates.append(preferred_port)

    for pattern in ['/dev/ttyACM*', '/dev/ttyUSB*']:
        found = sorted(globmod.glob(pattern))
        for p in found:
            if p not in candidates:
                candidates.append(p)

    if not candidates:
        raise RuntimeError(
            'No USB serial ports found. '
            'Check that the USB-RS485 adapter is connected.'
        )

    logger.info(f'Gripper port candidates: {candidates}')

    for port in candidates:
        try:
            g = Gripper(port, baudrate=baud, id=gripper_id)
            pos = g.get_gripper_value()
            if pos is not None and isinstance(pos, int) and pos >= 0:
                logger.info(f'Gripper found on {port} (position={pos})')
                return g, port
            else:
                g.close()
        except Exception as e:
            logger.debug(f'Port {port} failed: {e}')
            continue

    raise RuntimeError(
        f'Gripper not found on any port. Tried: {candidates}. '
        f'Check USB cable, adapter, and gripper power (24V).'
    )


# ═══════════════════════════════════════════════════════════════════════════════
#  GRIPPER NODE
# ═══════════════════════════════════════════════════════════════════════════════

class GripperNode(Node):

    def __init__(self):
        super().__init__('anubix_gripper')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('simulate',               False)
        self.declare_parameter('gripper_port',           'auto')
        self.declare_parameter('gripper_baud',           115200)
        self.declare_parameter('gripper_id',             14)
        self.declare_parameter('torque',                 5)
        self.declare_parameter('close_speed',            2)
        self.declare_parameter('open_speed',             20)
        self.declare_parameter('release_speed',          2)
        self.declare_parameter('position_threshold',     1)
        self.declare_parameter('stable_readings',        3)
        self.declare_parameter('sample_interval',        0.15)
        self.declare_parameter('max_detect_time',        12.0)
        self.declare_parameter('leaf_min_position',      1)
        self.declare_parameter('leaf_max_position',      99)
        self.declare_parameter('max_pick_tries',         5)
        self.declare_parameter('retry_delay',            1.5)
        self.declare_parameter('status_interval',        0.5)
        self.declare_parameter('grip_sim_delay',         1.0)

        self._simulate           = self.get_parameter('simulate').value
        self._port               = self.get_parameter('gripper_port').value
        self._baud               = self.get_parameter('gripper_baud').value
        self._gripper_id         = self.get_parameter('gripper_id').value
        self._torque             = self.get_parameter('torque').value
        self._close_speed        = self.get_parameter('close_speed').value
        self._open_speed         = self.get_parameter('open_speed').value
        self._release_speed      = self.get_parameter('release_speed').value
        self._pos_threshold      = self.get_parameter('position_threshold').value
        self._stable_readings    = self.get_parameter('stable_readings').value
        self._sample_interval    = self.get_parameter('sample_interval').value
        self._max_detect_time    = self.get_parameter('max_detect_time').value
        self._leaf_min_pos       = self.get_parameter('leaf_min_position').value
        self._leaf_max_pos       = self.get_parameter('leaf_max_position').value
        self._max_pick_tries     = self.get_parameter('max_pick_tries').value
        self._retry_delay        = self.get_parameter('retry_delay').value
        self._status_interval    = self.get_parameter('status_interval').value
        self._grip_sim_delay     = self.get_parameter('grip_sim_delay').value

        self._lock = threading.Lock()
        self._busy = False
        self._force_stopped = False
        self._holding_leaf = False

        # ── hardware connection ──────────────────────────────────────────────
        self.gripper = None
        self._connected_port = None

        if not self._simulate:
            try:
                self.gripper, self._connected_port = find_gripper_port(
                    self._port, self._baud, self._gripper_id, self.get_logger()
                )
                self.gripper.set_gripper_torque(self._torque)
                self.get_logger().info(
                    f'Gripper connected on {self._connected_port} '
                    f'(baud={self._baud}, id={self._gripper_id})'
                )
            except Exception as e:
                self.get_logger().error(f'Gripper connection failed: {e}')
                raise
        else:
            self.get_logger().info('simulate=true — no gripper hardware connection.')

        self._cb_group = ReentrantCallbackGroup()

        # ── SUPERVISOR INTERFACE (master node compatibility) ─────────────────
        self._gripper_status_pub = self.create_publisher(
            String, '/arm/gripper_status', _QOS_STATUS)
        self._touch_status_pub = self.create_publisher(
            Bool, '/arm/touch_status', _QOS_STATUS)

        self.create_subscription(
            Bool, '/supervisor/grip',
            self._on_supervisor_grip, _QOS_RELIABLE_VOL,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Bool, '/supervisor/force_stop',
            self._on_force_stop, _QOS_RELIABLE_VOL,
            callback_group=self._cb_group,
        )

        # ── DIRECT INTERFACE (manual control / gripper_sender) ───────────────
        self.status_pub   = self.create_publisher(String,  '/gripper/status',   10)
        self.position_pub = self.create_publisher(Float32, '/gripper/position', 10)

        self.create_subscription(
            String, '/gripper/command',
            self._command_callback, 10,
            callback_group=self._cb_group,
        )

        self.create_service(
            Trigger, '/gripper/pick',
            self._pick_callback, callback_group=self._cb_group)
        self.create_service(
            Trigger, '/gripper/open',
            self._open_callback, callback_group=self._cb_group)
        self.create_service(
            Trigger, '/gripper/close',
            self._close_callback, callback_group=self._cb_group)
        self.create_service(
            Trigger, '/gripper/release',
            self._release_callback, callback_group=self._cb_group)

        # ── status timer ─────────────────────────────────────────────────────
        self.create_timer(
            self._status_interval, self._publish_status,
            callback_group=self._cb_group,
        )

        # ── open on startup ──────────────────────────────────────────────────
        if not self._simulate:
            self._open_gripper()

        self.get_logger().info(
            f'GripperNode ready  [simulate={self._simulate}  '
            f'port={self._connected_port or "sim"}  '
            f'torque={self._torque}  close_speed={self._close_speed}]'
        )

    # ═══════════════════════════════════════════════════════════════════════
    #  SUPERVISOR CALLBACKS (master node bridge)
    # ═══════════════════════════════════════════════════════════════════════

    def _on_supervisor_grip(self, msg: Bool):
        """Bridge /supervisor/grip -> gripper hardware -> /arm/gripper_status + /arm/touch_status."""
        t = threading.Thread(
            target=self._execute_supervisor_grip, args=(msg.data,), daemon=True)
        t.start()

    def _execute_supervisor_grip(self, close: bool):
        if self._force_stopped:
            self._gripper_status_pub.publish(String(data='mechanical_error'))
            return

        if self._simulate:
            self._simulate_grip(close)
            return

        if close:
            if not self._acquire():
                self.get_logger().warn('Gripper busy — supervisor grip ignored')
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                return
            try:
                success, pos, msg_text = self._pick_leaf()
                if success:
                    self._holding_leaf = True
                    self._gripper_status_pub.publish(String(data='successful_grip'))
                    self._touch_status_pub.publish(Bool(data=True))
                else:
                    self._holding_leaf = False
                    self._gripper_status_pub.publish(String(data='mechanical_error'))
                    self._touch_status_pub.publish(Bool(data=False))
            except Exception as e:
                self.get_logger().error(f'Grip failed: {e}')
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                self._touch_status_pub.publish(Bool(data=False))
            finally:
                self._release_lock()
        else:
            if not self._acquire():
                self.get_logger().warn('Gripper busy — supervisor release ignored')
                self._gripper_status_pub.publish(String(data='mechanical_error'))
                return
            try:
                self._release_gripper()
                self._holding_leaf = False
                self._gripper_status_pub.publish(String(data='successful_release'))
                self._touch_status_pub.publish(Bool(data=False))
            except Exception as e:
                self.get_logger().error(f'Release failed: {e}')
                self._gripper_status_pub.publish(String(data='mechanical_error'))
            finally:
                self._release_lock()

    def _simulate_grip(self, close: bool):
        action = 'close' if close else 'open'
        self.get_logger().info(f'[SIM] gripper {action}')
        time.sleep(self._grip_sim_delay)
        if close:
            self._holding_leaf = True
            self._gripper_status_pub.publish(String(data='successful_grip'))
            self._touch_status_pub.publish(Bool(data=True))
        else:
            self._holding_leaf = False
            self._gripper_status_pub.publish(String(data='successful_release'))
            self._touch_status_pub.publish(Bool(data=False))

    def _on_force_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('FORCE STOP received.')
            self._force_stopped = True
            if self.gripper:
                try:
                    self.gripper.set_gripper_stop()
                except Exception:
                    pass
            self._release_lock()
        else:
            self.get_logger().info('Force stop cleared.')
            self._force_stopped = False

    # ═══════════════════════════════════════════════════════════════════════
    #  CORE GRIPPER ACTIONS
    # ═══════════════════════════════════════════════════════════════════════

    def _open_gripper(self):
        self.gripper.set_gripper_torque(self._torque)
        self.gripper.set_gripper_value(100, self._open_speed)
        time.sleep(2)
        self._publish_feedback("OPEN")

    def _release_gripper(self):
        self.gripper.set_gripper_torque(self._torque)
        self.gripper.set_gripper_value(100, self._release_speed)
        time.sleep(3)
        self._publish_feedback("RELEASED")

    def _close_fully(self):
        self.gripper.set_gripper_torque(self._torque)
        self.gripper.set_gripper_value(0, self._close_speed)
        time.sleep(4)
        self._publish_feedback("CLOSED")

    def _close_and_detect(self):
        """
        Ultra-sensitive leaf detection via position monitoring.
        Start closing slowly, sample position, detect when it stops moving.
        Returns (PickResult, final_position).
        """
        self.gripper.set_gripper_torque(self._torque)
        self.gripper.set_gripper_value(0, self._close_speed)

        prev_position  = 100
        stable_count   = 0
        elapsed        = 0.0

        self.get_logger().info('  Closing... monitoring position')

        while elapsed < self._max_detect_time:
            if self._force_stopped:
                return PickResult.FAILED, 0

            time.sleep(self._sample_interval)
            elapsed += self._sample_interval

            try:
                current_position = self.gripper.get_gripper_value()
            except Exception as e:
                self.get_logger().warn(f'  Position read failed: {e}')
                continue

            delta = abs(prev_position - current_position)

            if delta < self._pos_threshold:
                stable_count += 1
            else:
                stable_count = 0

            if int(elapsed * 10) % 10 == 0:
                self.get_logger().info(
                    f'  pos={current_position:>3}  delta={delta}  '
                    f'stable={stable_count}/{self._stable_readings}'
                )

            if stable_count >= self._stable_readings:
                if current_position > self._leaf_min_pos:
                    thickness_pct = 100 - current_position
                    self.get_logger().info(
                        f'  Stopped at position {current_position} '
                        f'(~{thickness_pct}% thickness detected)'
                    )
                    return PickResult.SUCCESS, current_position
                else:
                    return PickResult.EMPTY, current_position

            prev_position = current_position

        final = self.gripper.get_gripper_value()
        return PickResult.TIMEOUT, final

    def _pick_leaf(self, max_tries=None):
        """Full leaf pick sequence with retries. Returns (success, position, message)."""
        if max_tries is None:
            max_tries = self._max_pick_tries

        for attempt in range(1, max_tries + 1):
            self.get_logger().info(f'Pick attempt {attempt}/{max_tries}')
            self._publish_feedback(f'PICKING attempt:{attempt}/{max_tries}')

            result, position = self._close_and_detect()

            if result == PickResult.SUCCESS:
                msg = f'GRABBED position:{position}/100 thickness:{100 - position}%'
                self.get_logger().info(msg)
                self._publish_feedback(msg)
                return True, position, msg

            elif result == PickResult.FAILED:
                msg = f'ABORTED by force stop at attempt {attempt}'
                self.get_logger().warn(msg)
                self._publish_feedback(msg)
                self._open_gripper()
                return False, 0, msg

            elif result == PickResult.EMPTY:
                msg = f'EMPTY fully closed at {position} — retry {attempt}'
                self.get_logger().warn(msg)
                self._publish_feedback(msg)

            elif result == PickResult.TIMEOUT:
                msg = f'TIMEOUT at position {position} — retry {attempt}'
                self.get_logger().warn(msg)
                self._publish_feedback(msg)

            if attempt < max_tries:
                self._open_gripper()
                time.sleep(self._retry_delay)

        fail_msg = f'FAILED after {max_tries} attempts'
        self.get_logger().error(fail_msg)
        self._publish_feedback(fail_msg)
        self._open_gripper()
        return False, 0, fail_msg

    # ═══════════════════════════════════════════════════════════════════════
    #  BUSY GUARD
    # ═══════════════════════════════════════════════════════════════════════

    def _acquire(self):
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _release_lock(self):
        with self._lock:
            self._busy = False

    # ═══════════════════════════════════════════════════════════════════════
    #  TOPIC COMMAND CALLBACK (direct interface)
    # ═══════════════════════════════════════════════════════════════════════

    def _command_callback(self, msg):
        if self._simulate:
            self.get_logger().warn('simulate=true — gripper commands ignored')
            return

        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Command received: "{cmd}"')

        PICK_WORDS    = {"pick", "grab", "leaf", "gentle", "get", "catch"}
        OPEN_WORDS    = {"open", "reset"}
        RELEASE_WORDS = {"release", "drop", "let", "free"}
        CLOSE_WORDS   = {"close"}
        STOP_WORDS    = {"stop", "halt", "emergency"}
        STATUS_WORDS  = {"status", "check", "position", "where"}

        words = set(cmd.split())

        if words & PICK_WORDS:
            if not self._acquire():
                self.get_logger().warn('Gripper busy — ignoring pick command')
                return
            try:
                self._pick_leaf()
            finally:
                self._release_lock()

        elif words & RELEASE_WORDS:
            if not self._acquire():
                return
            try:
                self._release_gripper()
            finally:
                self._release_lock()

        elif words & OPEN_WORDS:
            if not self._acquire():
                return
            try:
                self._open_gripper()
            finally:
                self._release_lock()

        elif words & CLOSE_WORDS:
            if not self._acquire():
                return
            try:
                self._close_fully()
            finally:
                self._release_lock()

        elif words & STOP_WORDS:
            self.get_logger().warn('STOP command received!')
            try:
                self.gripper.set_gripper_stop()
                self._publish_feedback('STOPPED')
            except Exception as e:
                self.get_logger().error(f'Stop failed: {e}')
            self._release_lock()

        elif words & STATUS_WORDS:
            self._log_status()

        else:
            self.get_logger().warn(
                f'Unknown command: "{cmd}" — '
                'valid: pick | open | release | close | stop | status'
            )

    # ═══════════════════════════════════════════════════════════════════════
    #  SERVICE CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def _pick_callback(self, request, response):
        if self._simulate:
            response.success = False
            response.message = 'simulate=true — no hardware'
            return response
        if not self._acquire():
            response.success = False
            response.message = 'Gripper busy — try again'
            return response
        try:
            success, pos, msg = self._pick_leaf()
            response.success = success
            response.message = msg
        finally:
            self._release_lock()
        return response

    def _open_callback(self, request, response):
        if self._simulate:
            response.success = False
            response.message = 'simulate=true'
            return response
        if not self._acquire():
            response.success = False
            response.message = 'Gripper busy'
            return response
        try:
            self._open_gripper()
            response.success = True
            response.message = 'Gripper opened'
        finally:
            self._release_lock()
        return response

    def _close_callback(self, request, response):
        if self._simulate:
            response.success = False
            response.message = 'simulate=true'
            return response
        if not self._acquire():
            response.success = False
            response.message = 'Gripper busy'
            return response
        try:
            self._close_fully()
            response.success = True
            response.message = 'Gripper closed'
        finally:
            self._release_lock()
        return response

    def _release_callback(self, request, response):
        if self._simulate:
            response.success = False
            response.message = 'simulate=true'
            return response
        if not self._acquire():
            response.success = False
            response.message = 'Gripper busy'
            return response
        try:
            self._release_gripper()
            response.success = True
            response.message = 'Leaf released gently'
        finally:
            self._release_lock()
        return response

    # ═══════════════════════════════════════════════════════════════════════
    #  STATUS PUBLISHING
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_feedback(self, text):
        self.status_pub.publish(String(data=text))

    def _publish_status(self):
        if self._simulate:
            smsg = String()
            smsg.data = f'position:sim status:SIM node:{"BUSY" if self._busy else "IDLE"}'
            self.status_pub.publish(smsg)
            return

        try:
            pos    = self.gripper.get_gripper_value()
            raw    = self.gripper.get_gripper_status()
            label  = GRIPPER_HW_STATUS.get(raw, f"UNKNOWN({raw})")
            busy   = "BUSY" if self._busy else "IDLE"

            self.status_pub.publish(
                String(data=f'position:{pos} status:{label} node:{busy}'))
            self.position_pub.publish(Float32(data=float(pos)))
        except Exception:
            pass

    def _log_status(self):
        if self._simulate:
            self.get_logger().info('Status | simulate=true')
            return
        try:
            pos   = self.gripper.get_gripper_value()
            raw   = self.gripper.get_gripper_status()
            label = GRIPPER_HW_STATUS.get(raw, "UNKNOWN")
            self.get_logger().info(
                f'Status | position:{pos}/100  hw_status:{label}  busy:{self._busy}'
            )
        except Exception as e:
            self.get_logger().warn(f'Status read failed: {e}')


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — opening gripper...')
        if node.gripper:
            try:
                node._open_gripper()
            except Exception:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
