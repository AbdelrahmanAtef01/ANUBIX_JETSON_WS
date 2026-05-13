#!/usr/bin/env python3
"""
ANUBIX Supabase Uploader Node
==============================
Runs on the Jetson Orin Nano.

Triggered directly by the spectrometer: whenever /spectrometer/result is
published (success only), this node:
  1. Captures a photo from the USB camera (camera 2, index 0)
  2. Uploads the photo to Supabase Storage (bucket: plant-images)
  3. Builds a ReadingModel from the spectral JSON + photo URL
  4. Inserts the row into the Supabase 'readings' table

All steps run in a background thread. Up to max_retries attempts are made
for the DB insert. Every step is logged.

Topics subscribed:
  /spectrometer/result   std_msgs/String  (JSON AnalysisResult, success only)

Topics published:
  /supabase/upload_status  std_msgs/String  uploading | success | retrying | failure

Parameters (overridable via config YAML or env vars):
  robot_id           UUID of this robot
  task_id            UUID of the current task/mission
  plant_location     "x,y" grid coordinate
  usb_camera_index   USB camera index for photo capture (-1 = disabled)
  supabase_url       Supabase project URL  (or env: SUPABASE_URL)
  supabase_key       Supabase anon key     (or env: SUPABASE_KEY)
  max_retries        Upload attempts before giving up (default: 3)
  retry_delay_s      Seconds between retry attempts   (default: 2.0)

Env vars take priority over ROS params for credentials:
  SUPABASE_URL, SUPABASE_KEY
"""

import os
import json
import time
import threading
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from anubix_supabase.supabase_uploader import SupabaseUploader, ReadingModel

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

_DEFAULT_ROBOT_ID = '34a957fd-d45c-4dbf-8e02-be8e1b5e349a'
_DEFAULT_TASK_ID  = '40e4060b-5bc8-4044-9d71-046fee27a757'
_DEFAULT_SB_URL   = 'https://bdkutmmrcjckaazzzspe.supabase.co'
_DEFAULT_SB_KEY   = 'sb_publishable_VY6-Jjc6f20Wcbb3Rm8gwg_ZK6CYuh3'


class SupabaseUploaderNode(Node):

    def __init__(self):
        super().__init__('anubix_supabase_uploader')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_id',          _DEFAULT_ROBOT_ID)
        self.declare_parameter('task_id',           _DEFAULT_TASK_ID)
        self.declare_parameter('plant_location',    '0,0')
        self.declare_parameter('usb_camera_index',  0)
        self.declare_parameter('supabase_url',      _DEFAULT_SB_URL)
        self.declare_parameter('supabase_key',      _DEFAULT_SB_KEY)
        self.declare_parameter('max_retries',       3)
        self.declare_parameter('retry_delay_s',     2.0)

        self._robot_id        = self.get_parameter('robot_id').value
        self._task_id         = self.get_parameter('task_id').value
        self._plant_location  = self.get_parameter('plant_location').value
        self._usb_cam_index   = int(self.get_parameter('usb_camera_index').value)
        self._max_retries     = self.get_parameter('max_retries').value
        self._retry_delay     = self.get_parameter('retry_delay_s').value

        url = os.environ.get('SUPABASE_URL') or self.get_parameter('supabase_url').value
        key = os.environ.get('SUPABASE_KEY') or self.get_parameter('supabase_key').value
        creds_from_env = bool(os.environ.get('SUPABASE_URL') or os.environ.get('SUPABASE_KEY'))

        # ── State ─────────────────────────────────────────────────────────────
        self._stats = {'results_received': 0, 'uploads_ok': 0, 'uploads_failed': 0}
        self._lock = threading.Lock()

        # ── Supabase client ───────────────────────────────────────────────────
        try:
            self._uploader = SupabaseUploader(url=url, key=key)
            self.get_logger().info(
                f'[SUPABASE] Client ready — url={url!r}  '
                f'creds_source={"env_vars" if creds_from_env else "ros_params"}')
        except Exception as e:
            self._uploader = None
            self.get_logger().error(
                f'[SUPABASE] Client INIT FAILED: {type(e).__name__}: {e}  '
                f'Uploads disabled until fixed. Check supabase_url / supabase_key.')

        # ── QoS ───────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, '/spectrometer/result', self._on_spectral_result, reliable_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, '/supabase/upload_status', reliable_qos)

        cam_status = (
            f'index={self._usb_cam_index}'
            if self._usb_cam_index >= 0 and _CV2_AVAILABLE
            else 'DISABLED (set usb_camera_index >= 0 and install opencv)'
        )
        self.get_logger().info('=' * 62)
        self.get_logger().info('  ANUBIX Supabase Uploader Node')
        self.get_logger().info(f'  Listening on /spectrometer/result')
        self.get_logger().info(f'  robot_id       = {self._robot_id}')
        self.get_logger().info(f'  task_id        = {self._task_id}')
        self.get_logger().info(f'  location       = {self._plant_location!r}')
        self.get_logger().info(f'  photo capture  = {cam_status}')
        self.get_logger().info(f'  retries        = {self._max_retries}  delay={self._retry_delay}s')
        self.get_logger().info('=' * 62)

    # ── Subscriber callback ────────────────────────────────────────────────────

    def _on_spectral_result(self, msg: String):
        with self._lock:
            self._stats['results_received'] += 1
            total = self._stats['results_received']

        self.get_logger().info(
            f'[SUPABASE] /spectrometer/result received [total={total}] — '
            f'dispatching upload')

        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(
                f'[SUPABASE] Cannot parse JSON: {type(e).__name__}: {e}  '
                f'raw={msg.data!r}')
            self._status_pub.publish(String(data='failure'))
            return

        self.get_logger().info(
            f'[SUPABASE] Payload — '
            f'task_type={payload.get("task_type","?")!r}  '
            f'classification={payload.get("classification","?")!r}  '
            f'value={payload.get("value","?")}  '
            f'confidence={payload.get("confidence","?")}')

        # Prefer IDs supplied by OmniLink (forwarded through the
        # spectrometer payload). Fall back to the node parameters only if
        # the payload omitted them, so we never silently overwrite a real
        # robot/task UUID with a hardcoded default.
        payload_robot_id = (payload.get('robot_id') or '').strip()
        payload_task_id  = (payload.get('task_id')  or '').strip()
        robot_id = payload_robot_id or self._robot_id
        task_id  = payload_task_id  or self._task_id

        if not payload_robot_id or not payload_task_id:
            self.get_logger().warning(
                f'[SUPABASE] Spectrometer payload missing IDs '
                f'(robot_id={payload_robot_id!r}, task_id={payload_task_id!r}) — '
                f'falling back to node params robot_id={self._robot_id!r}, '
                f'task_id={self._task_id!r}')
        else:
            self.get_logger().info(
                f'[SUPABASE] Using IDs from spectrometer payload: '
                f'robot_id={robot_id!r} task_id={task_id!r}')

        context = {
            'robot_id':       robot_id,
            'task_id':        task_id,
            'plant_location': self._plant_location,
        }

        threading.Thread(
            target=self._upload_with_retry,
            args=(payload, context),
            daemon=True,
        ).start()

    # ── Photo capture ──────────────────────────────────────────────────────────

    def _capture_and_upload_photo(self) -> Optional[str]:
        """
        Capture one frame from the USB camera and upload it to Supabase Storage.
        Returns the public URL, or None if capture or upload fails.
        """
        if self._usb_cam_index < 0:
            self.get_logger().info('[SUPABASE] Photo capture disabled (usb_camera_index < 0)')
            return None

        if not _CV2_AVAILABLE:
            self.get_logger().warning(
                '[SUPABASE] opencv-python not installed — skipping photo capture. '
                'Run: pip3 install opencv-python')
            return None

        if self._uploader is None:
            self.get_logger().warning(
                '[SUPABASE] Uploader not ready — skipping photo capture')
            return None

        self.get_logger().info(
            f'[SUPABASE] Opening USB camera index={self._usb_cam_index} for photo capture')

        cap = cv2.VideoCapture(self._usb_cam_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(
                f'[SUPABASE] Cannot open USB camera at index {self._usb_cam_index}')
            return None

        try:
            # Flush stale buffered frames before grabbing
            for _ in range(5):
                cap.read()

            ret, frame = cap.read()
            if not ret or frame is None:
                self.get_logger().error('[SUPABASE] Failed to read frame from USB camera')
                return None

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            local_path = f'/tmp/anubix_scan_{timestamp}.jpg'
            cv2.imwrite(local_path, frame)
            self.get_logger().info(f'[SUPABASE] Photo saved locally: {local_path}')

        except Exception as e:
            self.get_logger().error(f'[SUPABASE] Camera error during capture: {e}')
            return None
        finally:
            cap.release()

        # Upload to Supabase Storage
        public_url = self._uploader.upload_image(local_path, bucket='plant-images')
        if public_url:
            self.get_logger().info(f'[SUPABASE] Photo uploaded → {public_url}')
        else:
            self.get_logger().warning(
                '[SUPABASE] Photo upload to storage failed — '
                'continuing with DB insert (no photo_url)')
        return public_url

    # ── Upload with retry ─────────────────────────────────────────────────────

    def _upload_with_retry(self, payload: dict, context: dict):
        if self._uploader is None:
            self.get_logger().error(
                '[SUPABASE] Uploader not initialized — '
                'fix credentials and restart the node.')
            self._status_pub.publish(String(data='failure'))
            with self._lock:
                self._stats['uploads_failed'] += 1
            return

        task_type      = payload.get('task_type', 'unknown')
        classification = payload.get('classification', 'unknown')
        value          = float(payload.get('value', 0.0))
        confidence     = float(payload.get('confidence', 0.0))

        robot_id       = context['robot_id']
        task_id        = context['task_id']
        plant_location = context['plant_location']

        # ── Step 1: capture plant photo from USB camera ───────────────────────
        self.get_logger().info('[SUPABASE] Step 1/2 — capturing plant photo from USB camera')
        photo_url = self._capture_and_upload_photo()

        # ── Step 2: build ReadingModel and insert into DB ─────────────────────
        self.get_logger().info(
            f'[SUPABASE] Step 2/2 — building ReadingModel: '
            f'task={task_type!r}  class={classification!r}  '
            f'value={value:.4f}  confidence={confidence:.2%}  '
            f'robot={robot_id}  task_id={task_id}  '
            f'location={plant_location!r}  '
            f'photo_url={photo_url!r}')

        reading = ReadingModel.from_spectral_result(
            robot_id=robot_id,
            task_id=task_id,
            plant_location=plant_location,
            task_type=task_type,
            classification=classification,
            value=value,
            photo_1_url=photo_url,
        )

        for attempt in range(1, self._max_retries + 1):
            self.get_logger().info(
                f'[SUPABASE] DB insert attempt {attempt}/{self._max_retries}')

            self._status_pub.publish(String(data='uploading'))
            success = self._uploader.upload_reading(reading)

            if success:
                with self._lock:
                    self._stats['uploads_ok'] += 1
                    ok_total = self._stats['uploads_ok']
                self.get_logger().info(
                    f'[SUPABASE] Upload SUCCESS on attempt {attempt}  '
                    f'[total ok={ok_total}  failed={self._stats["uploads_failed"]}]')
                self._status_pub.publish(String(data='success'))
                return

            if attempt < self._max_retries:
                self.get_logger().warning(
                    f'[SUPABASE] Attempt {attempt} FAILED — '
                    f'retrying in {self._retry_delay}s '
                    f'({self._max_retries - attempt} attempts left)')
                self._status_pub.publish(String(data='retrying'))
                time.sleep(self._retry_delay)

        # All retries exhausted
        with self._lock:
            self._stats['uploads_failed'] += 1
            fail_total = self._stats['uploads_failed']

        self.get_logger().error(
            f'[SUPABASE] ALL {self._max_retries} ATTEMPTS FAILED  '
            f'[total failures={fail_total}]  '
            f'LOST DATA — row not uploaded: {reading.to_dict()}')
        self._status_pub.publish(String(data='failure'))


def main(args=None):
    rclpy.init(args=args)
    node = SupabaseUploaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[SUPABASE] Uploader node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
