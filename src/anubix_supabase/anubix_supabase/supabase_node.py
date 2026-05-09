#!/usr/bin/env python3
"""
ANUBIX Supabase Uploader Node
==============================
Runs on the Jetson Orin Nano.

Triggered directly by the spectrometer: whenever /spectrometer/result
is published (which only happens on pipeline SUCCESS), this node
builds a ReadingModel from the JSON payload and inserts it into the
Supabase 'readings' table.

The upload runs in a background thread so the ROS executor is never
blocked. Up to `max_retries` attempts are made, with `retry_delay_s`
seconds between each attempt. Every step is logged.

Topics subscribed:
  /spectrometer/result   std_msgs/String  (JSON AnalysisResult, on success)

Topics published:
  /supabase/upload_status  std_msgs/String  uploading | success | retrying | failure

Parameters (all overridable via config YAML or env vars):
  robot_id         string  UUID of this robot
  task_id          string  UUID of the current task
  plant_location   string  "x,y" grid coordinate — update before each mission
  supabase_url     string  Supabase project URL  (or env: SUPABASE_URL)
  supabase_key     string  Supabase anon/publishable key (or env: SUPABASE_KEY)
  max_retries      int     Upload attempts before giving up  (default: 3)
  retry_delay_s    float   Seconds between retry attempts   (default: 2.0)

Environment variables take priority over ROS parameters for credentials:
  SUPABASE_URL, SUPABASE_KEY
"""

import os
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from anubix_supabase.supabase_uploader import SupabaseUploader, ReadingModel


_DEFAULT_ROBOT_ID = '34a957fd-d45c-4dbf-8e02-be8e1b5e349a'
_DEFAULT_TASK_ID  = '40e4060b-5bc8-4044-9d71-046fee27a757'
_DEFAULT_SB_URL   = 'https://bdkutmmrcjckaazzzspe.supabase.co'
_DEFAULT_SB_KEY   = 'sb_publishable_VY6-Jjc6f20Wcbb3Rm8gwg_ZK6CYuh3'


class SupabaseUploaderNode(Node):

    def __init__(self):
        super().__init__('anubix_supabase_uploader')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_id',       _DEFAULT_ROBOT_ID)
        self.declare_parameter('task_id',        _DEFAULT_TASK_ID)
        self.declare_parameter('plant_location', '0,0')
        self.declare_parameter('supabase_url',   _DEFAULT_SB_URL)
        self.declare_parameter('supabase_key',   _DEFAULT_SB_KEY)
        self.declare_parameter('max_retries',    3)
        self.declare_parameter('retry_delay_s',  2.0)

        self._robot_id       = self.get_parameter('robot_id').value
        self._task_id        = self.get_parameter('task_id').value
        self._plant_location = self.get_parameter('plant_location').value
        self._max_retries    = self.get_parameter('max_retries').value
        self._retry_delay    = self.get_parameter('retry_delay_s').value

        # Env vars override params for credentials
        url = os.environ.get('SUPABASE_URL') or self.get_parameter('supabase_url').value
        key = os.environ.get('SUPABASE_KEY') or self.get_parameter('supabase_key').value

        creds_from_env = bool(os.environ.get('SUPABASE_URL') or os.environ.get('SUPABASE_KEY'))

        # ── State ─────────────────────────────────────────────────────────────
        self._stats = {
            'results_received': 0,
            'uploads_ok': 0,
            'uploads_failed': 0,
        }
        self._lock = threading.Lock()

        # ── Supabase client ───────────────────────────────────────────────────
        try:
            self._uploader = SupabaseUploader(url=url, key=key)
            self.get_logger().info(
                f'[SUPABASE] Client ready  '
                f'url={url!r}  '
                f'creds_source={"env_vars" if creds_from_env else "ros_params"}')
        except Exception as e:
            self._uploader = None
            self.get_logger().error(
                f'[SUPABASE] Client INIT FAILED: {type(e).__name__}: {e}  '
                f'Uploads will be skipped until this is fixed. '
                f'Check supabase_url / supabase_key in params or env vars.')

        # ── QoS ───────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String,
            '/spectrometer/result',
            self._on_spectral_result,
            reliable_qos,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, '/supabase/upload_status', reliable_qos)

        self.get_logger().info('=' * 62)
        self.get_logger().info('  ANUBIX Supabase Uploader Node')
        self.get_logger().info(f'  Listening on /spectrometer/result')
        self.get_logger().info(
            f'  robot_id  = {self._robot_id}')
        self.get_logger().info(
            f'  task_id   = {self._task_id}')
        self.get_logger().info(
            f'  location  = {self._plant_location!r}  '
            f'(update via supabase_params.yaml before each mission)')
        self.get_logger().info(
            f'  retries   = {self._max_retries}  delay={self._retry_delay}s')
        self.get_logger().info('=' * 62)

    # ── Subscriber callback ────────────────────────────────────────────────────

    def _on_spectral_result(self, msg: String):
        """
        Called directly when /spectrometer/result arrives.
        Parses the JSON payload and fires an upload in a background thread.
        """
        with self._lock:
            self._stats['results_received'] += 1
            total = self._stats['results_received']

        self.get_logger().info(
            f'[SUPABASE] /spectrometer/result received  '
            f'[total received={total}]  — dispatching upload')

        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(
                f'[SUPABASE] Cannot parse /spectrometer/result JSON: '
                f'{type(e).__name__}: {e}  raw={msg.data!r}')
            self._status_pub.publish(String(data='failure'))
            return

        # Log what we received before any processing
        self.get_logger().info(
            f'[SUPABASE] Payload — '
            f'task_type={payload.get("task_type","?")!r}  '
            f'classification={payload.get("classification","?")!r}  '
            f'value={payload.get("value","?")}  '
            f'confidence={payload.get("confidence","?")}  '
            f'details={payload.get("details",{})}')

        # Snapshot mutable params before handing off to thread
        context = {
            'robot_id':       self._robot_id,
            'task_id':        self._task_id,
            'plant_location': self._plant_location,
        }

        threading.Thread(
            target=self._upload_with_retry,
            args=(payload, context),
            daemon=True,
        ).start()

    # ── Upload with retry ─────────────────────────────────────────────────────

    def _upload_with_retry(self, payload: dict, context: dict):
        if self._uploader is None:
            self.get_logger().error(
                '[SUPABASE] Uploader not initialized — '
                'fix supabase_url / supabase_key and restart the node.')
            self._status_pub.publish(String(data='failure'))
            with self._lock:
                self._stats['uploads_failed'] += 1
            return

        task_type      = payload.get('task_type', 'unknown')
        classification = payload.get('classification', 'unknown')
        value          = float(payload.get('value', 0.0))
        confidence     = float(payload.get('confidence', 0.0))
        details        = payload.get('details', {})

        robot_id       = context['robot_id']
        task_id        = context['task_id']
        plant_location = context['plant_location']

        self.get_logger().info(
            f'[SUPABASE] Building ReadingModel — '
            f'task={task_type!r}  class={classification!r}  '
            f'value={value:.4f}  confidence={confidence:.2%}  '
            f'robot={robot_id}  task_id={task_id}  '
            f'location={plant_location!r}')

        reading = ReadingModel.from_spectral_result(
            robot_id=robot_id,
            task_id=task_id,
            plant_location=plant_location,
            task_type=task_type,
            classification=classification,
            value=value,
        )

        for attempt in range(1, self._max_retries + 1):
            self.get_logger().info(
                f'[SUPABASE] Upload attempt {attempt}/{self._max_retries}  '
                f'table=readings')

            self._status_pub.publish(String(data='uploading'))
            success = self._uploader.upload_reading(reading)

            if success:
                with self._lock:
                    self._stats['uploads_ok'] += 1
                    ok_total = self._stats['uploads_ok']
                self.get_logger().info(
                    f'[SUPABASE] Upload SUCCESS on attempt {attempt}  '
                    f'[total ok={ok_total}  '
                    f'failed={self._stats["uploads_failed"]}]')
                self._status_pub.publish(String(data='success'))
                return

            if attempt < self._max_retries:
                self.get_logger().warning(
                    f'[SUPABASE] Attempt {attempt} FAILED — '
                    f'retrying in {self._retry_delay}s  '
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
            f'LOST DATA — row that was NOT uploaded: {reading.to_dict()}')
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
