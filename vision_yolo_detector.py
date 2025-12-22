#!/usr/bin/env python3
"""
ROS2 Node: Vision + YOLOv8 Object Detection con Orbbec Camera
Integrazione completa: Camera Orbbec → YOLOv8 → Publish detections 3D
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple
import json

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ YOLOv8 non disponibile. Installare: pip install ultralytics")


class VisionYOLODetector(Node):
    """
    Nodo ROS2 che:
    1. Sottoscrive ai topic della camera Orbbec (RGB + Depth)
    2. Esegue detection con YOLOv8
    3. Pubblica detections con coordinate 3D (usando depth)
    4. Pubblica immagini annotate per debug/visualizzazione
    """
    
    def __init__(self):
        super().__init__('vision_yolo_detector')
        
        # Parametri configurabili
        self.declare_parameter('camera_rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('yolo_model', 'yolov8n.pt')  # nano model (veloce)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('enable_visualization', True)
        
        # Ottieni parametri
        self.rgb_topic = self.get_parameter('camera_rgb_topic').value
        self.depth_topic = self.get_parameter('camera_depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        self.get_logger().info("Inizializzazione Vision YOLO Detector...")
        self.get_logger().info(f"  RGB Topic: {self.rgb_topic}")
        self.get_logger().info(f"  Depth Topic: {self.depth_topic}")
        self.get_logger().info(f"  YOLO Model: {self.yolo_model_name}")
        self.get_logger().info(f"  Confidence: {self.confidence_threshold}")
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Dati camera
        self.latest_rgb_image: Optional[np.ndarray] = None
        self.latest_depth_image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.rgb_timestamp = None
        self.depth_timestamp = None
        
        # YOLO model
        self.yolo_model = None
        if YOLO_AVAILABLE:
            try:
                self.get_logger().info(f"Caricamento modello YOLOv8: {self.yolo_model_name}")
                self.yolo_model = YOLO(self.yolo_model_name)
                self.get_logger().info("✅ Modello YOLO caricato con successo")
            except Exception as e:
                self.get_logger().error(f"❌ Errore caricamento YOLO: {e}")
                self.yolo_model = None
        else:
            self.get_logger().error("❌ YOLOv8 non disponibile. Installare: pip install ultralytics")
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.info_callback,
            10
        )
        
        # Publishers
        self.detections_3d_pub = self.create_publisher(
            String,  # JSON con detections 3D
            '/vision/detections_3d',
            10
        )
        
        if self.enable_viz:
            self.annotated_image_pub = self.create_publisher(
                Image,
                '/vision/annotated_image',
                10
            )
        
        # Timer per processing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_and_detect)
        
        # Statistiche
        self.frame_count = 0
        self.detection_count = 0
        
        self.get_logger().info("✅ Vision YOLO Detector avviato")
        self.get_logger().info(f"   Publishing rate: {self.publish_rate} Hz")
    
    def rgb_callback(self, msg: Image):
        """Callback per immagine RGB"""
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Errore conversione RGB: {e}")
    
    def depth_callback(self, msg: Image):
        """Callback per immagine depth"""
        try:
            # Depth tipicamente in mm (uint16) per Orbbec
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Errore conversione Depth: {e}")
    
    def info_callback(self, msg: CameraInfo):
        """Callback per camera info (intrinsics)"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("✅ Camera info ricevute")
            self.get_logger().info(f"   Resolution: {msg.width}x{msg.height}")
            self.get_logger().info(f"   Intrinsics: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}")
    
    def process_and_detect(self):
        """Processa immagine corrente e esegue detection"""
        if self.latest_rgb_image is None:
            if self.frame_count == 0:
                self.get_logger().warn("In attesa di immagini dalla camera...")
            return
        
        if self.yolo_model is None:
            return
        
        self.frame_count += 1
        
        try:
            # YOLO Detection
            results = self.yolo_model(
                self.latest_rgb_image,
                conf=self.confidence_threshold,
                verbose=False
            )
            
            # Processa risultati
            detections_3d = []
            
            if len(results) > 0:
                result = results[0]  # Primo frame
                boxes = result.boxes
                
                for box in boxes:
                    # Estrai info detection
                    xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = result.names[cls]
                    
                    # Centro del bounding box
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    
                    # Detection 3D (se depth disponibile)
                    if self.latest_depth_image is not None and self.camera_info is not None:
                        point_3d = self.get_3d_point(center_x, center_y)
                        if point_3d is not None:
                            detections_3d.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox_2d': {
                                    'x1': float(xyxy[0]),
                                    'y1': float(xyxy[1]),
                                    'x2': float(xyxy[2]),
                                    'y2': float(xyxy[3])
                                },
                                'position_3d': {
                                    'x': point_3d[0],
                                    'y': point_3d[1],
                                    'z': point_3d[2]
                                },
                                'center_2d': {
                                    'x': center_x,
                                    'y': center_y
                                }
                            })
            
            # Pubblica detections 3D (JSON)
            if detections_3d:
                msg_3d = String()
                msg_3d.data = json.dumps({
                    'timestamp': self.get_clock().now().seconds_nanoseconds(),
                    'frame_id': 'camera_color_optical_frame',
                    'detections': detections_3d
                })
                self.detections_3d_pub.publish(msg_3d)
                self.detection_count += len(detections_3d)
            
            # Pubblica immagine annotata
            if self.enable_viz and len(results) > 0:
                annotated_img = results[0].plot()  # YOLO annotation
                img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'camera_color_optical_frame'
                self.annotated_image_pub.publish(img_msg)
            
            # Log periodico
            if self.frame_count % 100 == 0:
                self.get_logger().info(
                    f"Stats: {self.frame_count} frames, "
                    f"{self.detection_count} detections totali, "
                    f"Current: {len(detections_3d)} objects"
                )
        
        except Exception as e:
            self.get_logger().error(f"Errore durante detection: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def get_3d_point(self, px: int, py: int) -> Optional[Tuple[float, float, float]]:
        """
        Converte pixel (x,y) + depth in punto 3D (X,Y,Z) in metri
        usando camera intrinsics
        """
        if self.latest_depth_image is None or self.camera_info is None:
            return None
        
        try:
            # Controlla bounds
            h, w = self.latest_depth_image.shape[:2]
            if px < 0 or px >= w or py < 0 or py >= h:
                return None
            
            # Ottieni depth (in mm tipicamente per Orbbec)
            depth_value = self.latest_depth_image[py, px]
            
            # Converti in metri
            if depth_value == 0 or depth_value > 10000:  # Invalid depth
                return None
            
            Z = depth_value / 1000.0  # mm → m
            
            # Camera intrinsics (K matrix)
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            # Back-projection: pixel → 3D
            X = (px - cx) * Z / fx
            Y = (py - cy) * Z / fy
            
            return (float(X), float(Y), float(Z))
        
        except Exception as e:
            self.get_logger().error(f"Errore calcolo 3D point: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    
    node = VisionYOLODetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutdown Vision YOLO Detector")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
