#!/bin/bash

# ================================================================================
# INSTALLA TUTTO DIRETTAMENTE SU AI ACCELERATOR
# Esegui questo script DIRETTAMENTE sull'AI Accelerator
# NON serve Windows, scarica tutto automaticamente
# ================================================================================

set -e

echo "=================================================================================="
echo "🚀 INSTALLAZIONE COMPLETA VISION SYSTEM"
echo "=================================================================================="
echo ""
echo "Questo script:"
echo "  1. Crea tutti i file necessari"
echo "  2. Installa tutte le dipendenze"
echo "  3. Configura tutto"
echo ""

# Verifica directory
if [ ! -d "remote_ur_control" ]; then
    echo "❌ Devi essere in ~/MekoAiAccelerator"
    echo "   cd ~/MekoAiAccelerator"
    exit 1
fi

echo "✅ Directory corretta: $(pwd)"
echo ""

read -p "Continuare con l'installazione? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installazione annullata"
    exit 0
fi

echo ""
echo "[1/4] Creazione file vision system..."
echo ""

# ========================================
# 1. vision_yolo_detector.py
# ========================================
echo "→ Creazione vision_yolo_detector.py..."

cat > vision_yolo_detector.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
ROS2 Node: Vision + YOLOv8 Object Detection con Orbecc Camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ YOLOv8 non disponibile")

class VisionYOLODetector(Node):
    def __init__(self):
        super().__init__('vision_yolo_detector')
        
        self.declare_parameter('camera_rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('enable_visualization', True)
        
        self.rgb_topic = self.get_parameter('camera_rgb_topic').value
        self.depth_topic = self.get_parameter('camera_depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        self.get_logger().info("Inizializzazione Vision YOLO Detector...")
        
        self.bridge = CvBridge()
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.camera_info = None
        
        if YOLO_AVAILABLE:
            try:
                self.yolo_model = YOLO(self.yolo_model_name)
                self.get_logger().info("✅ Modello YOLO caricato")
            except Exception as e:
                self.get_logger().error(f"❌ Errore YOLO: {e}")
                self.yolo_model = None
        else:
            self.yolo_model = None
        
        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, 10)
        
        self.detections_pub = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.detections_3d_pub = self.create_publisher(String, '/vision/detections_3d', 10)
        
        if self.enable_viz:
            self.annotated_image_pub = self.create_publisher(Image, '/vision/annotated_image', 10)
        
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.process_and_detect)
        
        self.frame_count = 0
        self.detection_count = 0
        
        self.get_logger().info("✅ Vision YOLO Detector avviato")
    
    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Errore RGB: {e}")
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Errore Depth: {e}")
    
    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("✅ Camera info ricevute")
    
    def process_and_detect(self):
        if self.latest_rgb_image is None or self.yolo_model is None:
            return
        
        self.frame_count += 1
        
        try:
            results = self.yolo_model(self.latest_rgb_image, conf=self.confidence_threshold, verbose=False)
            
            detections_3d = []
            
            if len(results) > 0:
                result = results[0]
                boxes = result.boxes
                
                for box in boxes:
                    xyxy = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = result.names[cls]
                    
                    center_x = int((xyxy[0] + xyxy[2]) / 2)
                    center_y = int((xyxy[1] + xyxy[3]) / 2)
                    
                    point_3d = self.get_3d_point(center_x, center_y)
                    if point_3d:
                        detections_3d.append({
                            'class': class_name,
                            'confidence': conf,
                            'bbox_2d': {'x1': float(xyxy[0]), 'y1': float(xyxy[1]), 'x2': float(xyxy[2]), 'y2': float(xyxy[3])},
                            'position_3d': {'x': point_3d[0], 'y': point_3d[1], 'z': point_3d[2]},
                            'center_2d': {'x': center_x, 'y': center_y}
                        })
            
            if detections_3d:
                msg_3d = String()
                msg_3d.data = json.dumps({
                    'timestamp': self.get_clock().now().seconds_nanoseconds(),
                    'frame_id': 'camera_color_optical_frame',
                    'detections': detections_3d
                })
                self.detections_3d_pub.publish(msg_3d)
                self.detection_count += len(detections_3d)
            
            if self.enable_viz and len(results) > 0:
                annotated_img = results[0].plot()
                img_msg = self.bridge.cv2_to_imgmsg(annotated_img, 'bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.annotated_image_pub.publish(img_msg)
            
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"Stats: {self.frame_count} frames, {self.detection_count} detections")
        
        except Exception as e:
            self.get_logger().error(f"Errore detection: {e}")
    
    def get_3d_point(self, px, py):
        if self.latest_depth_image is None or self.camera_info is None:
            return None
        
        try:
            h, w = self.latest_depth_image.shape[:2]
            if px < 0 or px >= w or py < 0 or py >= h:
                return None
            
            depth_value = self.latest_depth_image[py, px]
            if depth_value == 0 or depth_value > 10000:
                return None
            
            Z = depth_value / 1000.0
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            X = (px - cx) * Z / fx
            Y = (py - cy) * Z / fy
            
            return (float(X), float(Y), float(Z))
        except:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = VisionYOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOFPYTHON

chmod +x vision_yolo_detector.py
echo "  ✅ vision_yolo_detector.py"

# ========================================
# 2. vision_robot_coordinator.py (versione semplificata)
# ========================================
echo "→ Creazione vision_robot_coordinator.py..."

cat > vision_robot_coordinator.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
Vision Robot Coordinator - coordinatore semplificato
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VisionRobotCoordinator(Node):
    def __init__(self):
        super().__init__('vision_robot_coordinator')
        
        self.get_logger().info("Vision Robot Coordinator avviato")
        
        self.detections_sub = self.create_subscription(
            String, '/vision/detections_3d', self.detections_callback, 10)
        
        self.status_pub = self.create_publisher(String, '/vision/system_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.latest_detections = []
    
    def detections_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_detections = data.get('detections', [])
        except:
            pass
    
    def publish_status(self):
        status = {
            'state': 'running',
            'detections_count': len(self.latest_detections)
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionRobotCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOFPYTHON

chmod +x vision_robot_coordinator.py
echo "  ✅ vision_robot_coordinator.py"

# ========================================
# 3. Vision Web API
# ========================================
echo "→ Creazione vision_web_api.py..."

cat > remote_ur_control/vision_web_api.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
Vision Web API - integrazione web interface
"""

from flask import jsonify, request
import json
import time

class VisionWebAPI:
    def __init__(self, app=None):
        self.app = app
        self.latest_detections = []
        
        if app:
            self.register_routes(app)
    
    def register_routes(self, app):
        @app.route("/api/vision/status", methods=["GET"])
        def vision_status():
            return jsonify({
                'active': True,
                'detections_count': len(self.latest_detections)
            })
        
        @app.route("/api/vision/detections", methods=["GET"])
        def get_detections():
            return jsonify({
                'detections': self.latest_detections,
                'timestamp': time.time()
            })
    
    def update_detections(self, detections):
        self.latest_detections = detections

def add_vision_routes_to_app(app):
    vision_api = VisionWebAPI(app)
    return vision_api
EOFPYTHON

echo "  ✅ vision_web_api.py"

echo ""
echo "✅ File vision creati!"
echo ""

# ========================================
# 2. Installa dipendenze
# ========================================
echo "[2/4] Installazione dipendenze..."
echo ""

source /opt/ros/humble/setup.bash

echo "→ Python packages..."
pip3 install ultralytics opencv-python numpy flask requests --user -q

echo "→ ROS2 packages..."
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-msgs -qq

echo ""
echo "✅ Dipendenze installate!"
echo ""

# ========================================
# 3. Integra Vision API
# ========================================
echo "[3/4] Integrazione Vision API..."
echo ""

if ! grep -q "vision_web_api" remote_ur_control/web_interface.py 2>/dev/null; then
    echo "→ Integrazione in web_interface.py..."
    
    # Backup
    cp remote_ur_control/web_interface.py remote_ur_control/web_interface.py.backup
    
    # Aggiungi import dopo "from flask import"
    sed -i '/from flask import/a\\n# Vision API\\ntry:\\n    from .vision_web_api import add_vision_routes_to_app\\n    VISION_API_AVAILABLE = True\\nexcept:\\n    VISION_API_AVAILABLE = False' remote_ur_control/web_interface.py
    
    # Aggiungi inizializzazione dopo "app = Flask"
    sed -i '/app = Flask/a\\n# Initialize Vision API\\nif VISION_API_AVAILABLE:\\n    vision_api = add_vision_routes_to_app(app)' remote_ur_control/web_interface.py
    
    echo "  ✅ Vision API integrata"
else
    echo "  ✅ Vision API già integrata"
fi

echo ""

# ========================================
# 4. Crea script avvio
# ========================================
echo "[4/4] Creazione script avvio..."
echo ""

cat > avvia_vision_completo.sh << 'EOFBASH'
#!/bin/bash

echo "🚀 Avvio Vision System Completo"
echo ""

source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

export PYTHONPATH=$(pwd):$PYTHONPATH
export UR_ROBOT_IP="${UR_ROBOT_IP:-192.168.10.194}"

# Avvia camera se disponibile
if ros2 pkg list | grep -q orbbec_camera; then
    echo "→ Avvio camera..."
    ros2 launch orbbec_camera gemini_330_series.launch.py > /tmp/camera.log 2>&1 &
    sleep 3
fi

# Avvia vision detector
echo "→ Avvio vision detector..."
python3 vision_yolo_detector.py > /tmp/vision.log 2>&1 &
sleep 2

# Avvia coordinator
echo "→ Avvio coordinator..."
python3 vision_robot_coordinator.py > /tmp/coordinator.log 2>&1 &
sleep 1

echo ""
echo "✅ Vision system avviato!"
echo ""
echo "Log: /tmp/vision.log, /tmp/coordinator.log"
echo ""
echo "Avvio web interface..."
echo ""

python3 -m remote_ur_control.web_interface
EOFBASH

chmod +x avvia_vision_completo.sh
echo "  ✅ avvia_vision_completo.sh"

echo ""
echo "=================================================================================="
echo "✅ INSTALLAZIONE COMPLETATA!"
echo "=================================================================================="
echo ""
echo "Per avviare il sistema:"
echo "  ./avvia_vision_completo.sh"
echo ""
echo "Accesso da browser:"
echo "  http://$(hostname -I | awk '{print $1}'):8080"
echo ""
echo "Nuove API disponibili:"
echo "  GET /api/vision/status"
echo "  GET /api/vision/detections"
echo ""
echo "=================================================================================="




