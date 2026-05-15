#!/usr/bin/env python3
"""
ANUBIX Vision Node (Jetson Orin Nano)
======================================
Runs YOLO segmentation for leaf detection. Handles both RealSense (3D depth)
and USB mono camera (flange / parallax-based depth estimation).

Camera 1 (RealSense, base):
  Uses the full ``get_target_leaf`` scoring (left-half preference + middle
  penalty) to pick the best leaf on the plant. Provides 3D coordinates from
  hardware depth sensor.

Camera 2 (USB, flange-mounted on the arm):
  Picks the leaf closest to the gripper pixel position, then re-identifies
  the SAME leaf in a second frame (after a 1 cm right calibration move)
  using a nearest-centroid match so the calibration cannot lock onto a
  different leaf between frames. Calculates:
    - X, Y offset from horizontal displacement (calibration)
    - Z depth from vertical parallax disparity (parallax geometry)

Subscribes:
  /supervisor/perception_goal  (std_msgs/String)
  /supervisor/target_camera    (std_msgs/String)        "1"=RealSense, "2"=USB
  /supervisor/force_stop       (std_msgs/Bool)
  /arm/arm_status              (std_msgs/String)        "success" = move done
  /arm/current_pose            (geometry_msgs/PoseStamped)  latest arm pose

Publishes:
  /perception/status           (std_msgs/String)        "found" | "not_found"
  /perception/target_pose      (geometry_msgs/Pose)     3D offset (X, Y, Z)
  /supervisor/arm_nav_goal     (geometry_msgs/PoseStamped)  absolute calibration pose
"""

import sys
import time
import threading
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseStamped

try:
    import cv2
except ImportError:
    print('ERROR: opencv-python not installed. Run: pip3 install opencv-python')
    sys.exit(1)

try:
    from ultralytics import YOLO
except ImportError:
    print('ERROR: ultralytics not installed. Run: pip3 install ultralytics')
    sys.exit(1)

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

from anubix_vision.leaf_detection import (
    draw_grabber_ui,
    draw_hud,
    draw_leaves,
    get_closest_leaf_to_gripper,
    get_target_leaf,
    match_closest_leaf,
)


class VisionNode(Node):

    def __init__(self):
        super().__init__('anubix_vision')

        # Parameters
        self.declare_parameter('model_path', '../best.engine')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('usb_camera_index', 0)
        self.declare_parameter('visualize', False)
        # Wall-clock seconds to keep retrying detection before giving up.
        self.declare_parameter('detection_timeout_s', 30.0)
        # How many detection attempts per second while polling.
        self.declare_parameter('detection_rate_hz', 2.0)
        self.declare_parameter('arm_move_timeout_s', 30.0)
        # Pixel position of the gripper in the USB (flange) camera's frame.
        # -1 = use the frame centre at runtime. Camera 1 (RealSense) does
        # not see the gripper — it uses the bottom-centre grab plane
        # heuristic baked into get_target_leaf — so it has no gripper
        # parameter.
        self.declare_parameter('gripper_px_x_cam2', -1)
        self.declare_parameter('gripper_px_y_cam2', -1)
        # Maximum pixel distance allowed when re-identifying the leaf in
        # camera 2's second (post-calibration) frame.
        self.declare_parameter('tracking_max_dist_px', 200)
        # Calibration step size (metres). USB camera commands the arm to
        # move by exactly this much to the right.
        self.declare_parameter('calibration_step_m', 0.01)

        self._model_path = self.get_parameter('model_path').value
        self._confidence = float(self.get_parameter('confidence').value)
        self._usb_cam_index = int(self.get_parameter('usb_camera_index').value)
        self._visualize = bool(self.get_parameter('visualize').value)
        self._detection_timeout = float(self.get_parameter('detection_timeout_s').value)
        self._detection_rate = float(self.get_parameter('detection_rate_hz').value)
        self._arm_timeout = float(self.get_parameter('arm_move_timeout_s').value)
        self._gripper_px_cam2 = (
            int(self.get_parameter('gripper_px_x_cam2').value),
            int(self.get_parameter('gripper_px_y_cam2').value),
        )
        self._tracking_max_dist = int(self.get_parameter('tracking_max_dist_px').value)
        self._calibration_step_m = float(self.get_parameter('calibration_step_m').value)

        # State
        self._target_camera: int = 1
        self._force_stopped: bool = False
        self._arm_event = threading.Event()
        self._active_lock = threading.Lock()
        self._active: bool = False
        self._model = None
        self._waiting_for_arm: bool = False  # Only true when camera 2 requested arm move

        # Latest arm pose published by the arm node, used to build absolute
        # calibration goals instead of relative offsets.
        self._latest_arm_pose: PoseStamped = None
        self._arm_pose_lock = threading.Lock()

        self._sub_group = ReentrantCallbackGroup()

        # QoS
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
        # force_stop is edge-triggered — must be VOLATILE so a stale
        # latched True (e.g. from a previous rpi_bridge emergency stop)
        # cannot strand this node on every restart.
        force_stop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        self.create_subscription(
            String, '/supervisor/perception_goal', self._on_perception_goal,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/supervisor/target_camera', self._on_target_camera,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop,
            force_stop_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/arm/arm_status', self._on_arm_status,
            sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            PoseStamped, '/arm/current_pose', self._on_arm_pose,
            sub_qos, callback_group=self._sub_group)

        # Publishers
        self._pub_status = self.create_publisher(
            String, '/perception/status', sub_qos)
        self._pub_pose = self.create_publisher(
            Pose, '/perception/target_pose', sub_qos)
        self._pub_arm_goal = self.create_publisher(
            PoseStamped, '/supervisor/arm_nav_goal', cmd_qos)

        # Model load
        self._load_model()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ANUBIX Vision Node - Jetson Orin Nano')
        self.get_logger().info(f'  Model: {self._model_path}')
        self.get_logger().info(
            f'  RealSense SDK: '
            f'{"AVAILABLE" if REALSENSE_AVAILABLE else "NOT FOUND (camera 1 disabled)"}')
        self.get_logger().info(f'  USB camera index: {self._usb_cam_index}')
        self.get_logger().info(f'  Confidence threshold: {self._confidence}')
        self.get_logger().info(
            f'  Detection: up to {self._detection_timeout:.1f}s @ '
            f'{self._detection_rate:.1f} Hz')
        self.get_logger().info(f'  Visualize: {self._visualize}')
        self.get_logger().info(
            f'  Gripper pixel (cam2 flange): {self._gripper_px_cam2} '
            f'(<0 = frame centre)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('[VISION] Ready and waiting for goals.')

    def _load_model(self):
        try:
            self.get_logger().info(
                f'[VISION] Loading YOLO model: {self._model_path}')
            self._model = YOLO(self._model_path, task='segment')
            self.get_logger().info(
                f'[VISION] YOLO model loaded successfully: {self._model_path}')
        except Exception as exc:
            self.get_logger().error(
                f'[VISION] FAILED to load YOLO model: {exc}\n'
                f'{traceback.format_exc()}\n'
                f'[VISION] Export with: '
                f'yolo export model=best.pt format=engine half=true')
            self._model = None

    # Callbacks

    def _on_target_camera(self, msg: String):
        val = msg.data.strip()
        try:
            self._target_camera = int(val)
            self.get_logger().info(
                f'[VISION] Camera set -> {self._target_camera}')
        except ValueError:
            self.get_logger().warning(
                f'[VISION] Invalid target_camera value: "{val}" (not an int)')

    def _on_force_stop(self, msg: Bool):
        # Edge semantics: True aborts an in-flight pipeline (we wake the
        # arm event so any pending arm-confirmation wait exits immediately
        # and observes the new flag). False re-arms the node so the next
        # perception goal can run.
        was = self._force_stopped
        self._force_stopped = bool(msg.data)
        if self._force_stopped:
            self._arm_event.set()
            self.get_logger().warning(
                '[VISION] *** FORCE STOP *** — aborting pipeline')
        elif was:
            self.get_logger().info(
                '[VISION] Force stop CLEARED — ready for new goals')

    def _on_arm_status(self, msg: String):
        status = msg.data.strip().lower()
        self.get_logger().info(f'[VISION] /arm/arm_status = "{status}"')
        # Only process arm status if we're actively waiting for it (camera 2 calibration)
        if status == 'success' and self._waiting_for_arm:
            self._arm_event.set()
            self._waiting_for_arm = False
            self.get_logger().info(
                '[VISION] Arm calibration move CONFIRMED (camera 2)')
        elif status == 'success' and not self._waiting_for_arm:
            self.get_logger().debug(
                '[VISION] Ignoring arm status - not waiting for calibration')

    def _on_arm_pose(self, msg: PoseStamped):
        with self._arm_pose_lock:
            self._latest_arm_pose = msg

    def _on_perception_goal(self, msg: String):
        task_type = msg.data.strip().lower()

        self.get_logger().info(
            f'[VISION] ========================================')
        self.get_logger().info(
            f'[VISION] perception_goal RECEIVED: '
            f'task="{task_type}" camera={self._target_camera}')
        self.get_logger().info(
            f'[VISION] ========================================')

        if self._model is None:
            self.get_logger().error(
                '[VISION] Model NOT loaded — publishing "not_found". '
                'Check model_path parameter and TensorRT export.')
            self._pub_status.publish(String(data='not_found'))
            return

        with self._active_lock:
            if self._active:
                self.get_logger().warning(
                    '[VISION] Pipeline already running — ignoring duplicate goal')
                return
            self._active = True

        self._force_stopped = False

        threading.Thread(
            target=self._run_pipeline,
            args=(task_type, self._target_camera),
            daemon=True,
        ).start()

    def _run_pipeline(self, task_type: str, camera: int):
        try:
            self.get_logger().info(
                f'[VISION] Pipeline START — task="{task_type}" camera={camera}')
            start_time = time.time()

            if camera == 1:
                self._run_realsense(task_type)
            elif camera == 2:
                self._run_usb(task_type)
            else:
                self.get_logger().error(
                    f'[VISION] Unknown camera index: {camera}. '
                    f'Expected 1 (RealSense) or 2 (USB). Publishing "not_found".')
                self._pub_status.publish(String(data='not_found'))

            elapsed = time.time() - start_time
            self.get_logger().info(
                f'[VISION] Pipeline FINISHED in {elapsed:.1f}s')

        except Exception as exc:
            self.get_logger().error(
                f'[VISION] UNHANDLED exception in pipeline: {exc}\n'
                f'{traceback.format_exc()}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            with self._active_lock:
                self._active = False

    # Visualization helper

    def _show(self, window: str, frame):
        """Show frame if visualize=true. Safe to call when no display."""
        if not self._visualize:
            return
        try:
            cv2.imshow(window, frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.get_logger().debug(f'[VISION] imshow failed: {exc}')

    def _close_windows(self):
        if not self._visualize:
            return
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    # Time-bounded detection helpers

    def _detection_period(self) -> float:
        if self._detection_rate <= 0.0:
            return 0.5
        return 1.0 / self._detection_rate

    def _resolve_gripper(self, configured, width, height):
        """Return (gx, gy) clamped to the frame. -1 entries fall back to centre."""
        gx, gy = configured
        if gx < 0:
            gx = width // 2
        if gy < 0:
            gy = height // 2
        return int(max(0, min(gx, width - 1))), int(max(0, min(gy, height - 1)))

    # Camera 1: Intel RealSense

    def _run_realsense(self, task_type: str):
        if not REALSENSE_AVAILABLE:
            self.get_logger().error(
                '[VISION] RealSense SDK (pyrealsense2) NOT installed! '
                'Cannot use camera 1. Publishing "not_found".')
            self._pub_status.publish(String(data='not_found'))
            return

        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            self.get_logger().info('[VISION] Starting RealSense pipeline...')
            profile = pipeline.start(rs_config)
            align = rs.align(rs.stream.color)

            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            self.get_logger().info(
                f'[VISION] RealSense started — '
                f'intrinsics: {intrinsics.width}x{intrinsics.height} '
                f'fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f}')

            deadline = time.time() + self._detection_timeout
            period = self._detection_period()
            attempt = 0

            while time.time() < deadline:
                if self._force_stopped:
                    self.get_logger().warning(
                        '[VISION] Force stopped during RealSense capture')
                    self._pub_status.publish(String(data='not_found'))
                    return

                attempt += 1
                loop_start = time.time()

                try:
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                except RuntimeError as exc:
                    self.get_logger().warning(
                        f'[VISION] RealSense frame timeout (attempt {attempt}): {exc}')
                    self._sleep_remainder(loop_start, period)
                    continue

                aligned = align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()

                if not depth_frame or not color_frame:
                    self.get_logger().debug(
                        f'[VISION] No valid frame (attempt {attempt})')
                    self._sleep_remainder(loop_start, period)
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                h, w = color_image.shape[:2]

                results = self._model.predict(
                    color_image, conf=self._confidence, verbose=False)
                all_leaves, target_leaf = get_target_leaf(results, w, h)

                if self._visualize:
                    debug = color_image.copy()
                    draw_leaves(debug, all_leaves, target_leaf)
                    # Camera 1 does not see the gripper. The scoring uses
                    # the bottom-centre "grab plane" as its reference, so
                    # draw that instead of a gripper crosshair.
                    cv2.line(debug, (0, h - 20), (w, h - 20),
                             (200, 200, 0), 1)
                    cv2.circle(debug, (w // 2, h - 20), 6, (0, 200, 200), 2)
                    cv2.putText(debug, 'grab plane', (w // 2 + 10, h - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (0, 200, 200), 1)
                    remaining = max(0.0, deadline - time.time())
                    draw_hud(debug, [
                        f'CAM 1 (RealSense) task={task_type}',
                        f'attempt={attempt} t_left={remaining:0.1f}s',
                        f'leaves={len(all_leaves)} target={"yes" if target_leaf else "no"}',
                    ])
                    self._show('Anubix - Camera 1 (RealSense)', debug)

                if not target_leaf:
                    if attempt % 5 == 0:
                        self.get_logger().info(
                            f'[VISION] RealSense attempt {attempt}: '
                            f'no target leaf detected '
                            f'(t_left={max(0.0, deadline - time.time()):0.1f}s)')
                    self._sleep_remainder(loop_start, period)
                    continue

                cx, cy = target_leaf['centroid']
                dist = depth_frame.get_distance(cx, cy)

                if dist <= 0.0:
                    self.get_logger().debug(
                        f'[VISION] Invalid depth at ({cx},{cy}), retrying')
                    self._sleep_remainder(loop_start, period)
                    continue

                pt = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)
                x_m, y_m, z_m = float(pt[0]), float(pt[1]), float(pt[2])

                self.get_logger().info(
                    f'[VISION] TARGET LEAF FOUND! '
                    f'pixel=({cx},{cy}) '
                    f'3D=({x_m:.4f}, {y_m:.4f}, {z_m:.4f}) m '
                    f'depth={dist:.3f} m')

                pose = Pose()
                pose.position.x = x_m
                pose.position.y = y_m
                pose.position.z = z_m
                pose.orientation.w = 1.0
                self._pub_pose.publish(pose)
                self._pub_status.publish(String(data='found'))
                self.get_logger().info(
                    '[VISION] Published /perception/target_pose and '
                    '/perception/status="found"')

                if self._visualize:
                    final = color_image.copy()
                    draw_leaves(final, all_leaves, target_leaf)
                    draw_hud(final, [
                        f'CAM 1 FOUND task={task_type}',
                        f'pixel=({cx},{cy}) depth={dist:.2f}m',
                        f'3D=({x_m:.3f}, {y_m:.3f}, {z_m:.3f})',
                    ])
                    self._show('Anubix - Camera 1 (RealSense)', final)
                    time.sleep(1.0)

                return

            self.get_logger().warning(
                f'[VISION] No target found within {self._detection_timeout:.1f}s '
                f'on RealSense ({attempt} attempts). Publishing "not_found".')
            self._pub_status.publish(String(data='not_found'))

        except Exception as exc:
            self.get_logger().error(
                f'[VISION] RealSense exception: {exc}\n'
                f'{traceback.format_exc()}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            try:
                pipeline.stop()
                self.get_logger().info('[VISION] RealSense pipeline stopped')
            except Exception:
                pass
            self._close_windows()

    # Camera 2: USB flange — closest leaf to gripper, then re-identify

    def _run_usb(self, task_type: str):
        self.get_logger().info(
            f'[VISION] Opening USB camera at index {self._usb_cam_index}...')
        # CAP_V4L2 is Linux-only; default backend on other OSes.
        backend = getattr(cv2, 'CAP_V4L2', 0)
        cap = cv2.VideoCapture(self._usb_cam_index, backend)

        if not cap.isOpened():
            self.get_logger().error(
                f'[VISION] CANNOT open USB camera at index {self._usb_cam_index}! '
                f'Check: camera connected, permissions (/dev/video*), '
                f'not in use by another process.')
            self._pub_status.publish(String(data='not_found'))
            return

        try:
            for _ in range(10):
                cap.read()

            ret, probe = cap.read()
            if not ret:
                self.get_logger().error(
                    '[VISION] USB camera: first read FAILED.')
                self._pub_status.publish(String(data='not_found'))
                return

            h, w = probe.shape[:2]
            gx, gy = self._resolve_gripper(self._gripper_px_cam2, w, h)
            self.get_logger().info(
                f'[VISION] USB camera opened: {w}x{h} px, '
                f'gripper pixel=({gx},{gy})')

            # Phase 1: pick the leaf closest to the gripper pixel
            self.get_logger().info(
                '[VISION] === USB Phase 1: closest-leaf-to-gripper ===')
            centroid_1, leaf_1 = self._detect_phase1(cap, w, h, gx, gy, task_type)
            if centroid_1 is None:
                self.get_logger().warning(
                    f'[VISION] Phase 1 FAILED — no leaf within '
                    f'{self._detection_timeout:.1f}s. Publishing "not_found".')
                self._pub_status.publish(String(data='not_found'))
                return

            # Send absolute calibration goal: latest arm pose + 1 cm right (X)
            if not self._send_calibration_arm_goal():
                self._pub_status.publish(String(data='not_found'))
                return

            self.get_logger().info(
                f'[VISION] Waiting for /arm/arm_status="success" '
                f'(timeout={self._arm_timeout}s)...')
            if not self._arm_event.wait(timeout=self._arm_timeout):
                self.get_logger().error(
                    '[VISION] Arm move TIMED OUT — calibration failed!')
                self._pub_status.publish(String(data='not_found'))
                return

            if self._force_stopped:
                self.get_logger().warning(
                    '[VISION] Force stopped after arm move')
                self._pub_status.publish(String(data='not_found'))
                return

            self.get_logger().info(
                '[VISION] Arm move confirmed — proceeding to Phase 2')

            # Flush frames accumulated during arm motion
            for _ in range(5):
                cap.read()

            # Phase 2: re-identify the SAME leaf using nearest-centroid
            # matching to the Phase-1 centroid (with a sanity radius).
            self.get_logger().info(
                '[VISION] === USB Phase 2: re-identify same leaf ===')
            centroid_2, leaf_2 = self._detect_phase2(
                cap, w, h, gx, gy, centroid_1, task_type)

            if centroid_2 is None:
                self.get_logger().warning(
                    f'[VISION] Phase 2 FAILED — could not re-identify the leaf '
                    f'within {self._detection_timeout:.1f}s. '
                    f'Publishing "not_found".')
                self._pub_status.publish(String(data='not_found'))
                return

            # Pixels-per-cm calibration from the displacement of the SAME leaf
            dist_px = float(np.sqrt(
                (centroid_2[0] - centroid_1[0]) ** 2 +
                (centroid_2[1] - centroid_1[1]) ** 2
            ))

            calibration_cm = self._calibration_step_m * 100.0
            if dist_px < 1.0 or calibration_cm <= 0.0:
                self.get_logger().error(
                    '[VISION] Leaf did NOT move between frames! '
                    f'pixel_distance={dist_px:.2f}. Calibration invalid.')
                self._pub_status.publish(String(data='not_found'))
                return

            pixels_per_cm = dist_px / calibration_cm
            dx_px = centroid_2[0] - gx
            dy_px = centroid_2[1] - gy
            dx_cm = dx_px / pixels_per_cm
            dy_cm = dy_px / pixels_per_cm

            # Calculate depth using parallax from vertical shift between frames
            # When arm moves horizontally 1cm, vertical pixel shift relates to depth
            vertical_shift_px = abs(centroid_2[1] - centroid_1[1])

            # Using similar triangles: depth = (baseline × pixels_per_cm) / vertical_disparity_px
            # Baseline = calibration_cm (1.0 cm), disparity = vertical_shift_px
            if vertical_shift_px > 0.5:  # Need measurable shift for valid depth
                # Depth in cm from parallax geometry
                depth_cm = (calibration_cm * pixels_per_cm) / vertical_shift_px
                depth_m = depth_cm * 0.01
            else:
                # No vertical shift means leaf is very far or at optical axis
                # Fall back to assuming it's at gripper plane
                depth_m = 0.0
                self.get_logger().warning(
                    f'[VISION] Vertical shift too small ({vertical_shift_px:.2f}px) '
                    f'for depth calculation — assuming Z=0')

            self.get_logger().info(
                f'[VISION] Calibration: {calibration_cm:.2f} cm = '
                f'{dist_px:.2f} px  ->  1 cm = {pixels_per_cm:.2f} px')
            self.get_logger().info(
                f'[VISION] Offset from gripper: '
                f'dx={dx_cm:.2f} cm ({"Right" if dx_cm > 0 else "Left"}), '
                f'dy={dy_cm:.2f} cm ({"Down" if dy_cm > 0 else "Up"})')
            self.get_logger().info(
                f'[VISION] Depth calculation: vertical_shift={vertical_shift_px:.2f}px '
                f'-> depth={depth_cm if vertical_shift_px > 0.5 else 0.0:.2f} cm '
                f'({depth_m:.4f} m)')

            pose = Pose()
            pose.position.x = dx_cm * 0.01
            pose.position.y = -dy_cm * 0.01
            pose.position.z = depth_m
            pose.orientation.w = 1.0
            self._pub_pose.publish(pose)
            self._pub_status.publish(String(data='found'))
            self.get_logger().info(
                '[VISION] Published /perception/target_pose and '
                '/perception/status="found"')

            if self._visualize:
                ret, final = cap.read()
                if ret:
                    draw_leaves(final, [leaf_2] if leaf_2 else [], leaf_2)
                    draw_grabber_ui(final, gx, gy)
                    draw_hud(final, [
                        f'CAM 2 FOUND task={task_type}',
                        f'1 cm = {pixels_per_cm:.1f} px',
                        f'offset=({dx_cm:.2f}, {dy_cm:.2f}) cm',
                    ])
                    self._show('Anubix - Camera 2 (USB Flange)', final)
                    time.sleep(1.0)

        except Exception as exc:
            self.get_logger().error(
                f'[VISION] USB camera exception: {exc}\n'
                f'{traceback.format_exc()}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            cap.release()
            self.get_logger().info('[VISION] USB camera released')
            self._close_windows()

    def _detect_phase1(self, cap, w, h, gx, gy, task_type):
        deadline = time.time() + self._detection_timeout
        period = self._detection_period()
        attempt = 0

        while time.time() < deadline:
            if self._force_stopped:
                return None, None
            attempt += 1
            loop_start = time.time()

            ret, frame = cap.read()
            if not ret:
                self._sleep_remainder(loop_start, period)
                continue

            results = self._model.predict(
                frame, conf=self._confidence, verbose=False)
            all_leaves, target_leaf = get_closest_leaf_to_gripper(
                results, gx, gy)

            if self._visualize:
                debug = frame.copy()
                draw_leaves(debug, all_leaves, target_leaf)
                draw_grabber_ui(debug, gx, gy)
                if target_leaf:
                    cx, cy = target_leaf['centroid']
                    cv2.line(debug, (gx, gy), (cx, cy), (0, 255, 255), 1)
                remaining = max(0.0, deadline - time.time())
                draw_hud(debug, [
                    f'CAM 2 Phase 1 task={task_type}',
                    f'attempt={attempt} t_left={remaining:0.1f}s',
                    f'leaves={len(all_leaves)} pick=closest-to-gripper',
                ])
                self._show('Anubix - Camera 2 (USB Flange)', debug)

            if target_leaf:
                centroid = target_leaf['centroid']
                self.get_logger().info(
                    f'[VISION] Phase 1 COMPLETE — closest leaf at '
                    f'centroid=({centroid[0]}, {centroid[1]}) '
                    f'(took {attempt} attempts)')
                return centroid, target_leaf

            if attempt % 5 == 0:
                self.get_logger().info(
                    f'[VISION] Phase 1 attempt {attempt}: no leaf detected '
                    f'(t_left={max(0.0, deadline - time.time()):0.1f}s)')
            self._sleep_remainder(loop_start, period)

        return None, None

    def _detect_phase2(self, cap, w, h, gx, gy, anchor_centroid, task_type):
        deadline = time.time() + self._detection_timeout
        period = self._detection_period()
        attempt = 0

        while time.time() < deadline:
            if self._force_stopped:
                return None, None
            attempt += 1
            loop_start = time.time()

            ret, frame = cap.read()
            if not ret:
                self._sleep_remainder(loop_start, period)
                continue

            results = self._model.predict(
                frame, conf=self._confidence, verbose=False)
            all_leaves, matched, match_dist = match_closest_leaf(
                results, anchor_centroid, max_dist_px=self._tracking_max_dist)

            if self._visualize:
                debug = frame.copy()
                draw_leaves(debug, all_leaves, matched)
                draw_grabber_ui(debug, gx, gy)
                ax, ay = anchor_centroid
                cv2.circle(debug, (int(ax), int(ay)), 8, (0, 200, 255), 2)
                cv2.putText(debug, 'phase1_centroid', (int(ax) + 10, int(ay)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
                if matched:
                    cv2.line(debug, (int(ax), int(ay)),
                             matched['centroid'], (0, 200, 255), 1)
                remaining = max(0.0, deadline - time.time())
                draw_hud(debug, [
                    f'CAM 2 Phase 2 task={task_type}',
                    f'attempt={attempt} t_left={remaining:0.1f}s',
                    f'leaves={len(all_leaves)} match_dist={match_dist:.1f}px '
                    f'(max={self._tracking_max_dist}px)',
                ])
                self._show('Anubix - Camera 2 (USB Flange)', debug)

            if matched:
                self.get_logger().info(
                    f'[VISION] Phase 2 COMPLETE — same leaf re-identified at '
                    f'centroid={matched["centroid"]} '
                    f'(match_dist={match_dist:.1f}px, attempt={attempt})')
                return matched['centroid'], matched

            if attempt % 5 == 0:
                self.get_logger().info(
                    f'[VISION] Phase 2 attempt {attempt}: no acceptable match '
                    f'(nearest={match_dist:.1f}px > {self._tracking_max_dist}px)')
            self._sleep_remainder(loop_start, period)

        return None, None

    def _send_calibration_arm_goal(self) -> bool:
        """Publish an ABSOLUTE arm pose = (latest pose) + step_m on +X.

        Returns False if we have no recent arm pose to base the move on.
        """
        with self._arm_pose_lock:
            latest = self._latest_arm_pose

        self._arm_event.clear()
        self._waiting_for_arm = True  # Mark that we're expecting arm confirmation

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'base_link'

        if latest is None:
            self.get_logger().warning(
                '[VISION] No /arm/current_pose yet — sending calibration as '
                'a relative move in the "calibration" frame as a fallback.')
            ps.header.frame_id = 'calibration'
            ps.pose.position.x = self._calibration_step_m
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
        else:
            ps.pose.position.x = latest.pose.position.x + self._calibration_step_m
            ps.pose.position.y = latest.pose.position.y
            ps.pose.position.z = latest.pose.position.z
            ps.pose.orientation = latest.pose.orientation
            if ps.pose.orientation.w == 0.0 and ps.pose.orientation.x == 0.0 \
                    and ps.pose.orientation.y == 0.0 and ps.pose.orientation.z == 0.0:
                ps.pose.orientation.w = 1.0

        self._pub_arm_goal.publish(ps)
        self.get_logger().info(
            f'[VISION] Published calibration arm_nav_goal '
            f'(absolute={"yes" if latest is not None else "no, fallback"}): '
            f'x={ps.pose.position.x:.4f} y={ps.pose.position.y:.4f} '
            f'z={ps.pose.position.z:.4f}')
        return True

    @staticmethod
    def _sleep_remainder(loop_start: float, period: float):
        elapsed = time.time() - loop_start
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('[VISION] Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
