import os
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from .detector import YoloV5Detector


def load_class_names(path: Path) -> List[str]:
    with path.open('r', encoding='utf-8') as f:
        return [line.strip() for line in f.readlines() if line.strip()]


class YoloV5DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('yolov5_detector_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('weights_path', '')
        self.declare_parameter('class_names_path', '')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('use_cuda', False)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_topic', '/camera/color/yolov5_annotated')

        weights_path = self._resolve_share_path(
            self.get_parameter('weights_path').get_parameter_value().string_value,
            'models/yolov5n.onnx',
        )
        class_names_path = self._resolve_share_path(
            self.get_parameter('class_names_path').get_parameter_value().string_value,
            'models/coco_labels.txt',
        )
        conf_threshold = float(self.get_parameter('confidence_threshold').value)
        iou_threshold = float(self.get_parameter('iou_threshold').value)
        use_cuda = bool(self.get_parameter('use_cuda').value)

        if not Path(weights_path).exists():
            self.get_logger().error(f'Weights file not found: {weights_path}')
            raise FileNotFoundError(weights_path)
        if not Path(class_names_path).exists():
            self.get_logger().error(f'Class labels file not found: {class_names_path}')
            raise FileNotFoundError(class_names_path)

        class_names = load_class_names(Path(class_names_path))
        self.detector = YoloV5Detector(
            weights_path=weights_path,
            class_names=class_names,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            use_cuda=use_cuda,
        )

        self.bridge = CvBridge()
        image_topic = self.get_parameter('image_topic').value
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, qos)

        detections_topic = 'detections'
        self.detections_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self.publish_annotated = bool(self.get_parameter('publish_annotated_image').value)
        if self.publish_annotated:
            annotated_topic = self.get_parameter('annotated_topic').value
            self.annotated_pub = self.create_publisher(
                Image,
                annotated_topic,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        else:
            self.annotated_pub = None

        self.get_logger().info(
            f'YOLOv5 detector node ready. Subscribing to {image_topic}, detections on {detections_topic}'
        )
        self.get_logger().info(f'Weights: {weights_path}')
        self.get_logger().info(f'CUDA enabled: {use_cuda}')

    def _resolve_share_path(self, configured_path: str, default_relative: str) -> str:
        if configured_path:
            return configured_path
        share_dir = Path(get_package_share_directory('yolov5_detector'))
        return str(share_dir / default_relative)

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        detections = self.detector.infer(frame)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for det in detections:
            detection_msg = Detection2D()
            detection_msg.header = msg.header
            detection_msg.bbox.center.position.x = float(det.center[0])
            detection_msg.bbox.center.position.y = float(det.center[1])
            detection_msg.bbox.size_x = float(det.bbox[2])
            detection_msg.bbox.size_y = float(det.bbox[3])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det.class_name
            hypothesis.hypothesis.score = float(det.confidence)
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

        if detection_array.detections:
            self.detections_pub.publish(detection_array)

        if self.publish_annotated and self.annotated_pub:
            annotated = frame.copy()
            annotated = YoloV5Detector.draw_detections(annotated, detections)
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.annotated_pub.publish(out_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down YOLOv5 detector node.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloV5DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
