#!/usr/bin/env python3
"""
ANUBIX Vision Node
==================
Runs on the Jetson Orin Nano. Receives a perception goal from the master node,
acquires the target leaf with YOLO segmentation, and publishes the 3D / relative
position back to the master so the arm can reach it.

Subscribes:
  /supervisor/perception_goal  (std_msgs/String)      task_type, e.g. "disease"
  /supervisor/target_camera    (std_msgs/String)       "1" = RealSense, "2" = USB
  /supervisor/force_stop       (std_msgs/Bool)         abort immediately
  /arm/arm_status              (std_msgs/String)       "success" = calibration move done

Publishes:
  /perception/status           (std_msgs/String)       "found" | "not_found"
  /perception/target_pose      (geometry_msgs/Pose)    leaf position in metres
  /supervisor/arm_nav_goal     (geometry_msgs/PoseStamped)  calibration: 1 cm right

Camera 1 (RealSense D4xx):
  Single capture → YOLO → depth lookup → rs2_deproject_pixel_to_point → 3-D Pose.

Camera 2 (USB mono, V4L2):
  Phase 1: detect leaf centroid_1.
  Calibration: publish arm_nav_goal (frame_id="calibration", x=0.01 m = 1 cm right).
  Wait for /arm/arm_status: success (timeout = arm_move_timeout_s).
  Phase 2: detect leaf centroid_2.
  pixels_per_cm = pixel distance between centroids (arm moved exactly 1 cm).
  dx_cm, dy_cm = offset of centroid_2 from grabber centre / pixels_per_cm.
  Publish Pose(x=dx_cm*0.01, y=-dy_cm*0.01, z=0).
"""

import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
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

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('model_path', '../best.engine')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('usb_camera_index', 0)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('detection_max_attempts', 30)
        self.declare_parameter('arm_move_timeout_s', 30.0)

        self._model_path      = self.get_parameter('model_path').value
        self._confidence      = float(self.get_parameter('confidence').value)
        self._usb_cam_index   = int(self.get_parameter('usb_camera_index').value)
        self._show_preview    = bool(self.get_parameter('show_preview').value)
        self._max_attempts    = int(self.get_parameter('detection_max_attempts').value)
        self._arm_timeout     = float(self.get_parameter('arm_move_timeout_s').value)

        # ── State ─────────────────────────────────────────────────────────────
        self._target_camera: int   = 1
        self._force_stopped: bool  = False
        self._arm_event            = threading.Event()
        self._active_lock          = threading.Lock()
        self._active: bool         = False
        self._model                = None

        # ── QoS ───────────────────────────────────────────────────────────────
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

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, '/supervisor/perception_goal', self._on_perception_goal, cmd_qos)
        self.create_subscription(
            String, '/supervisor/target_camera', self._on_target_camera, cmd_qos)
        self.create_subscription(
            Bool, '/supervisor/force_stop', self._on_force_stop, sub_qos)
        self.create_subscription(
            String, '/arm/arm_status', self._on_arm_status, sub_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_status   = self.create_publisher(String,      '/perception/status',         sub_qos)
        self._pub_pose     = self.create_publisher(Pose,        '/perception/target_pose',    sub_qos)
        self._pub_arm_goal = self.create_publisher(PoseStamped, '/supervisor/arm_nav_goal',   cmd_qos)

        # ── Model load (synchronous — must complete before accepting goals) ──
        self._load_model()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ANUBIX Vision Node — Jetson Orin Nano')
        self.get_logger().info(f'  Model  : {self._model_path}')
        self.get_logger().info(f'  RealSense SDK: {"available" if REALSENSE_AVAILABLE else "NOT found — camera 1 disabled"}')
        self.get_logger().info('=' * 60)

    # ── Model ─────────────────────────────────────────────────────────────────

    def _load_model(self):
        try:
            self._model = YOLO(self._model_path, task='segment')
            self.get_logger().info(f'[VISION] YOLO model loaded: {self._model_path}')
        except Exception as exc:
            self.get_logger().error(f'[VISION] Failed to load YOLO model: {exc}')
            self.get_logger().error(
                '[VISION] Export the .pt with: '
                'yolo export model=best.pt format=engine half=true')
            self._model = None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_target_camera(self, msg: String):
        val = msg.data.strip()
        try:
            self._target_camera = int(val)
            self.get_logger().info(f'[VISION] Camera set → {self._target_camera}')
        except ValueError:
            self.get_logger().warning(f'[VISION] Invalid target_camera value: "{val}"')

    def _on_force_stop(self, msg: Bool):
        if msg.data:
            self._force_stopped = True
            self._arm_event.set()          # unblock any calibration wait immediately
            self.get_logger().warning('[VISION] Force stop — aborting pipeline')

    def _on_arm_status(self, msg: String):
        status = msg.data.strip().lower()
        self.get_logger().debug(f'[VISION] /arm/arm_status = {status}')
        if status == 'success':
            self._arm_event.set()
            self.get_logger().info('[VISION] Arm calibration move confirmed')

    def _on_perception_goal(self, msg: String):
        task_type = msg.data.strip().lower()
        self.get_logger().info(
            f'[VISION] perception_goal received: task="{task_type}" camera={self._target_camera}')

        if self._model is None:
            self.get_logger().error('[VISION] Model not loaded — publishing not_found')
            self._pub_status.publish(String(data='not_found'))
            return

        with self._active_lock:
            if self._active:
                self.get_logger().warning(
                    '[VISION] Pipeline already active — ignoring duplicate goal')
                return
            self._active = True

        self._force_stopped = False

        threading.Thread(
            target=self._run_pipeline,
            args=(task_type, self._target_camera),
            daemon=True,
        ).start()

    # ── Top-level pipeline dispatcher ─────────────────────────────────────────

    def _run_pipeline(self, task_type: str, camera: int):
        try:
            self.get_logger().info(
                f'[VISION] Pipeline start — task="{task_type}" camera={camera}')

            if camera == 1:
                self._run_realsense(task_type)
            elif camera == 2:
                self._run_usb(task_type)
            else:
                self.get_logger().error(f'[VISION] Unknown camera index: {camera}')
                self._pub_status.publish(String(data='not_found'))

        except Exception as exc:
            self.get_logger().error(f'[VISION] Unhandled pipeline exception: {exc}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            with self._active_lock:
                self._active = False
            self.get_logger().info('[VISION] Pipeline finished')

    # ── Camera 1: Intel RealSense ─────────────────────────────────────────────

    def _run_realsense(self, task_type: str):
        if not REALSENSE_AVAILABLE:
            self.get_logger().error('[VISION] RealSense SDK not installed — publishing not_found')
            self._pub_status.publish(String(data='not_found'))
            return

        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            profile = pipeline.start(rs_config)
            align   = rs.align(rs.stream.color)

            color_stream = profile.get_stream(rs.stream.color)
            intrinsics   = color_stream.as_video_stream_profile().get_intrinsics()

            self.get_logger().info('[VISION] RealSense pipeline started')

            for attempt in range(self._max_attempts):
                if self._force_stopped:
                    self.get_logger().warning('[VISION] Force stopped during RealSense capture')
                    self._pub_status.publish(String(data='not_found'))
                    return

                try:
                    frames = pipeline.wait_for_frames(timeout_ms=3000)
                except RuntimeError as exc:
                    self.get_logger().warning(f'[VISION] RealSense frame timeout: {exc}')
                    continue

                aligned      = align.process(frames)
                depth_frame  = aligned.get_depth_frame()
                color_frame  = aligned.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                h, w        = color_image.shape[:2]

                results = self._model.predict(
                    color_image, conf=self._confidence, verbose=False)
                all_leaves, target_leaf = get_target_leaf(results, w, h)

                if not target_leaf:
                    self.get_logger().debug(
                        f'[VISION] RealSense attempt {attempt + 1}/{self._max_attempts}: no target')
                    continue

                cx, cy = target_leaf['centroid']
                dist   = depth_frame.get_distance(cx, cy)

                if dist <= 0.0:
                    self.get_logger().debug(
                        f'[VISION] Invalid depth at ({cx},{cy}), retrying')
                    continue

                pt = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)
                x_m, y_m, z_m = float(pt[0]), float(pt[1]), float(pt[2])

                self.get_logger().info(
                    f'[VISION] TARGET LEAF pixel=({cx},{cy}) '
                    f'3D=({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m depth={dist:.3f} m')

                pose = Pose()
                pose.position.x    = x_m
                pose.position.y    = y_m
                pose.position.z    = z_m
                pose.orientation.w = 1.0
                self._pub_pose.publish(pose)
                self._pub_status.publish(String(data='found'))

                if self._show_preview:
                    draw_leaves(color_image, all_leaves, target_leaf)
                    draw_grabber_ui(color_image, w // 2, h // 2)
                    cv2.putText(color_image, f'Depth: {dist:.2f} m',
                                (cx - 50, cy + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.imshow('Anubix - RealSense', color_image)
                    cv2.waitKey(1)
                    cv2.destroyAllWindows()

                return  # success

            self.get_logger().warning(
                f'[VISION] No target found after {self._max_attempts} RealSense attempts')
            self._pub_status.publish(String(data='not_found'))

        except Exception as exc:
            self.get_logger().error(f'[VISION] RealSense exception: {exc}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            if self._show_preview:
                cv2.destroyAllWindows()

    # ── Camera 2: USB mono with calibration ───────────────────────────────────

    def _run_usb(self, task_type: str):
        cap = cv2.VideoCapture(self._usb_cam_index, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error(
                f'[VISION] Cannot open USB camera at index {self._usb_cam_index}')
            self._pub_status.publish(String(data='not_found'))
            return

        try:
            # Flush stale buffered frames
            for _ in range(10):
                cap.read()

            ret, probe = cap.read()
            if not ret:
                self.get_logger().error('[VISION] USB camera: first read failed')
                self._pub_status.publish(String(data='not_found'))
                return

            h, w          = probe.shape[:2]
            grabber_x     = w // 2
            grabber_y     = h // 2

            # ── Phase 1: first centroid ────────────────────────────────────
            self.get_logger().info('[VISION] USB Phase 1 — searching for initial leaf position')
            centroid_1 = None

            for attempt in range(self._max_attempts):
                if self._force_stopped:
                    self.get_logger().warning('[VISION] Force stopped during USB Phase 1')
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
                        f'[VISION] Phase 1 complete — centroid_1={centroid_1}')

                    if self._show_preview:
                        disp = frame.copy()
                        draw_leaves(disp, all_leaves, target_leaf)
                        draw_grabber_ui(disp, grabber_x, grabber_y)
                        cv2.imshow('Anubix - USB (Phase 1)', disp)
                        cv2.waitKey(1)

                    break

                self.get_logger().debug(
                    f'[VISION] USB Phase 1 attempt {attempt + 1}/{self._max_attempts}: no target')

            if centroid_1 is None:
                self.get_logger().warning(
                    f'[VISION] Phase 1 failed — no leaf in {self._max_attempts} attempts')
                self._pub_status.publish(String(data='not_found'))
                return

            # ── Calibration arm move: 1 cm right ──────────────────────────
            self.get_logger().info(
                '[VISION] Sending calibration arm move: 1 cm right (frame_id=calibration)')
            self._arm_event.clear()

            ps = PoseStamped()
            ps.header.stamp        = self.get_clock().now().to_msg()
            ps.header.frame_id     = 'calibration'
            ps.pose.position.x     = 0.01   # 1 cm
            ps.pose.position.y     = 0.0
            ps.pose.position.z     = 0.0
            ps.pose.orientation.w  = 1.0
            self._pub_arm_goal.publish(ps)

            self.get_logger().info(
                f'[VISION] Waiting for arm confirmation (timeout={self._arm_timeout} s)')
            if not self._arm_event.wait(timeout=self._arm_timeout):
                self.get_logger().error(
                    '[VISION] Arm move timed out — cannot complete calibration')
                self._pub_status.publish(String(data='not_found'))
                return

            if self._force_stopped:
                self.get_logger().warning('[VISION] Force stopped after arm move')
                self._pub_status.publish(String(data='not_found'))
                return

            self.get_logger().info('[VISION] Arm move confirmed — starting Phase 2')

            # Flush frames accumulated during arm motion
            for _ in range(5):
                cap.read()

            # ── Phase 2: second centroid ───────────────────────────────────
            self.get_logger().info('[VISION] USB Phase 2 — searching for leaf after arm move')
            centroid_2 = None

            for attempt in range(self._max_attempts):
                if self._force_stopped:
                    self.get_logger().warning('[VISION] Force stopped during USB Phase 2')
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
                        f'[VISION] Phase 2 complete — centroid_2={centroid_2}')

                    if self._show_preview:
                        disp = frame.copy()
                        draw_leaves(disp, all_leaves, target_leaf)
                        draw_grabber_ui(disp, grabber_x, grabber_y)
                        cv2.imshow('Anubix - USB (Phase 2)', disp)
                        cv2.waitKey(1)

                    break

                self.get_logger().debug(
                    f'[VISION] USB Phase 2 attempt {attempt + 1}/{self._max_attempts}: no target')

            if centroid_2 is None:
                self.get_logger().warning(
                    f'[VISION] Phase 2 failed — no leaf in {self._max_attempts} attempts after arm move')
                self._pub_status.publish(String(data='not_found'))
                return

            # ── Pixels-per-cm calibration ──────────────────────────────────
            dist_px = float(np.sqrt(
                (centroid_2[0] - centroid_1[0]) ** 2 +
                (centroid_2[1] - centroid_1[1]) ** 2
            ))

            if dist_px < 1.0:
                self.get_logger().error(
                    '[VISION] Leaf did not move between frames — calibration invalid. '
                    'Ensure the arm actually moved and the camera field of view is correct.')
                self._pub_status.publish(String(data='not_found'))
                return

            pixels_per_cm = dist_px                    # 1 cm = dist_px pixels
            dx_px  = centroid_2[0] - grabber_x
            dy_px  = centroid_2[1] - grabber_y
            dx_cm  = dx_px / pixels_per_cm
            dy_cm  = dy_px / pixels_per_cm

            self.get_logger().info(
                f'[VISION] Calibration: 1 cm = {pixels_per_cm:.2f} px | '
                f'offset from grabber: dx={dx_cm:.2f} cm, dy={dy_cm:.2f} cm')
            self.get_logger().info(
                f'[VISION] Grabber movement needed: '
                f'{"Right" if dx_cm > 0 else "Left"} {abs(dx_cm):.2f} cm, '
                f'{"Down" if dy_cm > 0 else "Up"} {abs(dy_cm):.2f} cm')

            # Publish pose: convert cm → m; image Y is inverted relative to robot Y
            pose = Pose()
            pose.position.x    = dx_cm * 0.01
            pose.position.y    = -dy_cm * 0.01
            pose.position.z    = 0.0
            pose.orientation.w = 1.0
            self._pub_pose.publish(pose)
            self._pub_status.publish(String(data='found'))

        except Exception as exc:
            self.get_logger().error(f'[VISION] USB camera exception: {exc}')
            self._pub_status.publish(String(data='not_found'))
        finally:
            cap.release()
            if self._show_preview:
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
