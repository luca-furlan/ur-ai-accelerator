import math
from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel


class Yolov5XYZNode(Node):
    def __init__(self) -> None:
        super().__init__('yolov5_xyz_node')

        self.declare_parameter('detections_topic', 'detections')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect')
        self.declare_parameter('depth_qos_reliability', 'best_effort')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('output_topic', 'detections_xyz')
        self.declare_parameter('max_depth', 3.0)
        self.declare_parameter('min_depth', 0.2)

        detections_topic = self.get_parameter('detections_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_depth = float(self.get_parameter('min_depth').value)

        reliability = self.get_parameter('depth_qos_reliability').value
        if reliability.lower() == 'reliable':
            depth_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        else:
            depth_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False
        self.depth_image: Optional[np.ndarray] = None
        self.depth_stamp = None

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback, depth_qos)
        self.create_subscription(Detection2DArray, detections_topic, self.detections_callback, 10)

        self.pose_pub = self.create_publisher(PoseArray, output_topic, 10)

        self.get_logger().info(
            'YOLOv5 XYZ node ready. Detections: %s, Depth: %s, Output: %s',
            detections_topic,
            depth_topic,
            output_topic,
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info('Camera intrinsics received: fx=%.2f, fy=%.2f', self.camera_model.fx(), self.camera_model.fy())

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = depth
            self.depth_stamp = msg.header.stamp
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error('Failed to convert depth image: %s', exc)

    def detections_callback(self, msg: Detection2DArray) -> None:
        if not self.camera_info_received:
            self.get_logger().warn_once('Camera intrinsics not yet received, skipping detections.')
            return
        if self.depth_image is None:
            self.get_logger().warn_once('No depth image available yet, skipping detections.')
            return

        pose_array = PoseArray()
        pose_array.header = msg.header

        for detection in msg.detections:
            if detection.bbox.size_x <= 0 or detection.bbox.size_y <= 0:
                continue

            u = int(round(detection.bbox.center.position.x))
            v = int(round(detection.bbox.center.position.y))

            if not (0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]):
                continue

            depth = self.depth_image[v, u]
            if np.isnan(depth) or depth <= 0:
                continue

            if depth.dtype == np.uint16:
                depth_m = float(depth) / 1000.0
            else:
                depth_m = float(depth)

            if depth_m < self.min_depth or depth_m > self.max_depth:
                continue

            ray = self.camera_model.projectPixelTo3dRay((u, v))
            ray = np.array(ray, dtype=np.float64)
            ray_norm = np.linalg.norm(ray)
            if ray_norm == 0:
                continue
            ray = ray / ray_norm
            point_cam = ray * depth_m

            pose = Pose()
            pose.position = Point(*point_cam)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            pose_array.poses.append(pose)

        if pose_array.poses:
            self.pose_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = Yolov5XYZNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
