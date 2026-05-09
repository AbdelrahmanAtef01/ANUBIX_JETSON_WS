#!/usr/bin/env python3
"""
ANUBIX Perception Stack - Placeholder Node
============================================
Subscribes to: /supervisor/perception_goal (std_msgs/String)
               /supervisor/target_camera (std_msgs/String)
Publishes to:  /perception/status (std_msgs/String)
               /perception/target_pose (geometry_msgs/Pose)

Status values: found | not_found

TODO: Integrate actual perception pipeline:
      - Camera driver (CSI / USB cameras on Jetson)
      - YOLO or custom crop detection model (TensorRT on Jetson GPU)
      - Depth estimation for target_pose (stereo or depth camera)
      - Camera switching logic (camera 1 = wide, camera 2 = close-up)
      - Water stress detection via NDVI/thermal imaging
      - Disease classification model
      - Harvest readiness assessment
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class PerceptionNode(Node):
    """
    Placeholder perception node.
    Receives perception goals and camera switch commands from the master.
    Publishes detection status and target pose for the arm stack.
    """

    def __init__(self):
        super().__init__('anubix_perception')

        # Parameters
        self.declare_parameter('simulate', True)
        self.declare_parameter('detection_delay', 1.5)
        self.declare_parameter('default_target_x', 0.5)
        self.declare_parameter('default_target_y', 0.0)
        self.declare_parameter('default_target_z', 0.2)
        self._simulate = self.get_parameter('simulate').value
        self._detection_delay = self.get_parameter('detection_delay').value

        self._active_camera = 1
        self._current_goal = ''

        # QoS
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.create_subscription(
            String, '/supervisor/perception_goal', self._on_perception_goal, cmd_qos)
        self.create_subscription(
            String, '/supervisor/target_camera', self._on_target_camera, cmd_qos)

        # Publishers
        self._status_pub = self.create_publisher(String, '/perception/status', pub_qos)
        self._pose_pub = self.create_publisher(Pose, '/perception/target_pose', pub_qos)

        self.get_logger().info('[PERCEPTION] Perception node ready (placeholder mode)')

    def _on_target_camera(self, msg: String):
        try:
            self._active_camera = int(msg.data)
        except ValueError:
            self._active_camera = 1
        self.get_logger().info(f'[PERCEPTION] Camera switched to: {self._active_camera}')

    def _on_perception_goal(self, msg: String):
        task_type = msg.data
        self._current_goal = task_type
        self.get_logger().info(
            f'[PERCEPTION] Goal received: {task_type} (camera {self._active_camera})')

        if self._simulate:
            # TODO: Replace with actual inference pipeline
            self.create_timer(
                self._detection_delay,
                lambda: self._complete_detection(task_type),
            )
        else:
            # TODO: Run detection model, publish result
            pass

    def _complete_detection(self, task_type: str):
        # TODO: Replace with actual model inference result
        self._status_pub.publish(String(data='found'))
        self.get_logger().info(f'[PERCEPTION] Detected: {task_type} = found')

        # Publish a target pose for the arm stack
        # TODO: Compute actual 3D pose from depth camera + detection bbox
        pose = Pose()
        pose.position.x = self.get_parameter('default_target_x').value
        pose.position.y = self.get_parameter('default_target_y').value
        pose.position.z = self.get_parameter('default_target_z').value
        pose.orientation.w = 1.0
        self._pose_pub.publish(pose)
        self.get_logger().info(
            f'[PERCEPTION] Published target_pose: '
            f'({pose.position.x}, {pose.position.y}, {pose.position.z})')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
