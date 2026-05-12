#!/usr/bin/env python3
"""
ANUBIX Vision Node (Jetson Orin Nano)
======================================
Runs YOLO segmentation for leaf detection. Handles both RealSense (3D depth)
and USB mono camera (calibration-based distance measurement).

Subscribes:
  /supervisor/perception_goal  (std_msgs/String)       task_type, e.g. "disease"
  /supervisor/target_camera    (std_msgs/String)       "1" = RealSense, "2" = USB
  /supervisor/force_stop       (std_msgs/Bool)         abort immediately
  /arm/arm_status              (std_msgs/String)       "success" = calibration move done

Publishes:
  /perception/status           (std_msgs/String)       "found" | "not_found"
  /perception/target_pose      (geometry_msgs/Pose)    leaf position in metres
  /supervisor/arm_nav_goal     (geometry_msgs/PoseStamped)  calibration: 1 cm right
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

from anubix_vision.leaf_detection import draw_grabber_ui, draw_leaves, get_target_leaf


class VisionNode(Node):

    def __init__(self):
        super().__init__('anubix_vision')

        # Parameters
        self.declare_parameter('model_path', '../best.engine')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('usb_camera_index', 0)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('detection_max_attempts', 30)
        self.declare_parameter('arm_move_timeout_s', 30.0)

        self._model_path = self.get_parameter('model_path').value
        self._confidence = float(self.get_parameter('confidence').value)
        self._usb_cam_index = int(self.get_parameter('usb_camera_index').value)
        self._show_preview = bool(self.get_parameter('show_preview').value)
        self._max_attempts = int(self.get_parameter('detection_max_attempts').value)
        self._arm_timeout = float(self.get_parameter('arm_move_timeout_s').value)

        # State
        self._target_camera: int = 1
        self._force_stopped: bool = False
        self._arm_event = threading.Event()
        self._active_lock = threading.Lock()
        self._active: bool = False
        self._model = None

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

        # Subscribers
        self.create_subscription(
            String, '/supervisor/perception_goal', self._on_perception_goal,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/supervisor/target_camera', self._on_target_camera,
            cmd_qos, callback_group=self._sub_group)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop,
            sub_qos, callback_group=self._sub_group)
        self.create_subscription(
            String, '/arm/arm_status', self._on_arm_status,
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
        self.get_logger().info(f'  Max detection attempts: {self._max_attempts}')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            '[VISION] Subscribed to /supervisor/perception_goal, '
            '/supervisor/target_camera, /supervisor/force_stop, /arm/arm_status')
        self.get_logger().info(
            '[VISION] Publishing on /perception/status, /perception/target_pose')
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
        if msg.data:
            self._force_stopped = True
            self._arm_event.set()
            self.get_logger().warning(
                '[VISION] *** FORCE STOP *** — aborting pipeline')

    def _on_arm_status(self, msg: String):
        status = msg.data.strip().lower()
        self.get_logger().info(f'[VISION] /arm/arm_status = "{status}"')
        if status == 'success':
            self._arm_event.set()
            self.get_logger().info(
                '[VISION] Arm calibration move CONFIRMED')

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

            for attempt in range(self._max_attempts):
                if self._force_stopped:
                    self.get_logger().warning(
                        '[VISION] Force stopped during RealSense capture')
                    self._pub_status.publish(String(data='not_found'))
                    return

                try:
                    frames = pipeline.wait_for_frames(timeout_ms=3000)
                except RuntimeError as exc:
                    self.get_logger().warning(
                        f'[VISION] RealSense frame timeout (attempt {attempt+1}): {exc}')
                    continue

                aligned = align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()

                if not depth_frame or not color_frame:
                    self.get_logger().debug(
                        f'[VISION] No valid frame (attempt {attempt+1})')
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                h, w = color_image.shape[:2]

                results = self._model.predict(
                    color_image, conf=self._confidence, verbose=False)
                all_leaves, target_leaf = get_target_leaf(results, w, h)

                if not target_leaf:
                    if (attempt + 1) % 5 == 0:
                        self.get_logger().info(
                            f'[VISION] RealSense attempt {attempt+1}/{self._max_attempts}: '
                            f'no target leaf detected')
                    continue

                cx, cy = target_leaf['centroid']
                dist = depth_frame.get_distance(cx, cy)

                if dist <= 0.0:
                    self.get_logger().debug(
                        f'[VISION] Invalid depth at ({cx},{cy}), retrying')
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

                if self._show_preview:
                    draw_leaves(color_image, all_leaves, target_leaf)
                    draw_grabber_ui(color_image, w // 2, h // 2)
                    cv2.imshow('Anubix - RealSense', color_image)
                    cv2.waitKey(1)
                    cv2.destroyAllWindows()

                return

            self.get_logger().warning(
                f'[VISION] No target found after {self._max_attempts} '
                f'RealSense attempts. Publishing "not_found".')
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
            if self._show_preview:
                cv2.destroyAllWindows()

    # Camera 2: USB mono with calibration

    def _run_usb(self, task_type: str):
        self.get_logger().info(
            f'[VISION] Opening USB camera at index {self._usb_cam_index}...')
        cap = cv2.VideoCapture(self._usb_cam_index, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error(
                f'[VISION] CANNOT open USB camera at index {self._usb_cam_index}! '
                f'Check: camera connected, permissions (/dev/video*), '
                f'not in use by another process.')
            self._pub_status.publish(String(data='not_found'))
            return

        try:
            # Flush stale buffered frames
            for _ in range(10):
                cap.read()

            ret, probe = cap.read()
            if not ret:
                self.get_logger().error(
                    '[VISION] USB camera: first read FAILED. '
                    'Camera may be disconnected or in error state.')
                self._pub_status.publish(String(data='not_found'))
                return

            h, w = probe.shape[:2]
            grabber_x = w // 2
            grabber_y = h // 2
            self.get_logger().info(
                f'[VISION] USB camera opened: {w}x{h} px, '
                f'grabber center=({grabber_x},{grabber_y})')

            # Phase 1: first centroid
            self.get_logger().info(
                '[VISION] === USB Phase 1: searching for initial leaf ===')
            centroid_1 = None

            for attempt in range(self._max_attempts):
                if self._force_stopped:
                    self.get_logger().warning(
                        '[VISION] Force stopped during USB Phase 1')
                    self._pub_status.publish(String(data='not_found'))
                    return

                ret, frame = cap.read()
                if not ret:
                    continue

                results = self._model.predict(
                    frame, conf=self._confidence, verbose=False)
                all_leaves, target_leaf = get_target_leaf(results, w, h)

                if target_leaf:
                    centroid_1 = target_leaf['centroid']
                    self.get_logger().info(
                        f'[VISION] Phase 1 COMPLETE — '
                        f'centroid_1=({centroid_1[0]}, {centroid_1[1]})')
                    break

                if (attempt + 1) % 5 == 0:
                    self.get_logger().info(
                        f'[VISION] Phase 1 attempt {attempt+1}/{self._max_attempts}: '
                        f'no target')

            if centroid_1 is None:
                self.get_logger().warning(
                    f'[VISION] Phase 1 FAILED — no leaf found in '
                    f'{self._max_attempts} attempts. Publishing "not_found".')
                self._pub_status.publish(String(data='not_found'))
                return

            # Calibration arm move: 1 cm right
            self.get_logger().info(
                '[VISION] === Calibration: sending arm 1 cm right ===')
            self._arm_event.clear()

            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'calibration'
            ps.pose.position.x = 0.01
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            self._pub_arm_goal.publish(ps)
            self.get_logger().info(
                f'[VISION] Published calibration arm_nav_goal. '
                f'Waiting for /arm/arm_status="success" '
                f'(timeout={self._arm_timeout}s)...')

            if not self._arm_event.wait(timeout=self._arm_timeout):
                self.get_logger().error(
                    '[VISION] Arm move TIMED OUT — calibration failed! '
                    'Check: arm_node running and responding.')
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

            # Phase 2: second centroid
            self.get_logger().info(
                '[VISION] === USB Phase 2: searching for leaf after arm move ===')
            centroid_2 = None

            for attempt in range(self._max_attempts):
                if self._force_stopped:
                    self.get_logger().warning(
                        '[VISION] Force stopped during USB Phase 2')
                    self._pub_status.publish(String(data='not_found'))
                    return

                ret, frame = cap.read()
                if not ret:
                    continue

                results = self._model.predict(
                    frame, conf=self._confidence, verbose=False)
                all_leaves, target_leaf = get_target_leaf(results, w, h)

                if target_leaf:
                    centroid_2 = target_leaf['centroid']
                    self.get_logger().info(
                        f'[VISION] Phase 2 COMPLETE — '
                        f'centroid_2=({centroid_2[0]}, {centroid_2[1]})')
                    break

                if (attempt + 1) % 5 == 0:
                    self.get_logger().info(
                        f'[VISION] Phase 2 attempt {attempt+1}/{self._max_attempts}: '
                        f'no target')

            if centroid_2 is None:
                self.get_logger().warning(
                    f'[VISION] Phase 2 FAILED — no leaf in '
                    f'{self._max_attempts} attempts after arm move. '
                    f'Publishing "not_found".')
                self._pub_status.publish(String(data='not_found'))
                return

            # Pixels-per-cm calibration
            dist_px = float(np.sqrt(
                (centroid_2[0] - centroid_1[0]) ** 2 +
                (centroid_2[1] - centroid_1[1]) ** 2
            ))

            if dist_px < 1.0:
                self.get_logger().error(
                    '[VISION] Leaf did NOT move between frames! '
                    f'pixel_distance={dist_px:.2f} (< 1.0). '
                    'Calibration invalid. '
                    'Check: arm actually moved, camera FOV correct.')
                self._pub_status.publish(String(data='not_found'))
                return

            pixels_per_cm = dist_px
            dx_px = centroid_2[0] - grabber_x
            dy_px = centroid_2[1] - grabber_y
            dx_cm = dx_px / pixels_per_cm
            dy_cm = dy_px / pixels_per_cm

            self.get_logger().info(
                f'[VISION] Calibration: 1 cm = {pixels_per_cm:.2f} px')
            self.get_logger().info(
                f'[VISION] Offset from grabber: '
                f'dx={dx_cm:.2f} cm ({"Right" if dx_cm > 0 else "Left"}), '
                f'dy={dy_cm:.2f} cm ({"Down" if dy_cm > 0 else "Up"})')

            pose = Pose()
            pose.position.x = dx_cm * 0.01
            pose.position.y = -dy_cm * 0.01
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            self._pub_pose.publish(pose)
            self._pub_status.publish(String(data='found'))
            self.get_logger().info(
                '[VISION] Published /perception/target_pose and '
                '/perception/status="found"')

        except Exception as exc:
            self.get_logger().error(
                f'[VISION] USB camera exception: {exc}\n'
                f'{traceback.format_exc()}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            cap.release()
            self.get_logger().info('[VISION] USB camera released')
            if self._show_preview:
                cv2.destroyAllWindows()


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
