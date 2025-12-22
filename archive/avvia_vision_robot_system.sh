#!/bin/bash

# ========================================
# Avvio Sistema Vision + Robot Integrato
# Camera Orbecc → YOLOv8 → MoveIt2 → Robot Control
# ========================================

set -e

echo "=========================================="
echo "AVVIO SISTEMA VISION + ROBOT"
echo "=========================================="
echo ""

# Colori per output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Configurazione
ROBOT_IP="${ROBOT_IP:-192.168.10.194}"
AUTO_MODE="${AUTO_MODE:-false}"
YOLO_MODEL="${YOLO_MODEL:-yolov8n.pt}"
CONFIDENCE="${CONFIDENCE:-0.5}"

echo -e "${GREEN}Configurazione:${NC}"
echo "  Robot IP: $ROBOT_IP"
echo "  Auto Mode: $AUTO_MODE"
echo "  YOLO Model: $YOLO_MODEL"
echo "  Confidence: $CONFIDENCE"
echo ""

# ========================================
# 1. Verifica prerequisiti
# ========================================

echo -e "${YELLOW}[1/5] Verifica prerequisiti...${NC}"

# ROS2
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}❌ ROS2 non trovato${NC}"
    echo "   Esegui: source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "   ✅ ROS2 disponibile"

# Python packages
python3 -c "import rclpy" 2>/dev/null || {
    echo -e "${RED}❌ rclpy non installato${NC}"
    exit 1
}
echo "   ✅ rclpy OK"

python3 -c "from ultralytics import YOLO" 2>/dev/null || {
    echo -e "${RED}❌ YOLOv8 non installato${NC}"
    echo "   Installa: pip install ultralytics"
    exit 1
}
echo "   ✅ YOLOv8 OK"

python3 -c "import cv2" 2>/dev/null || {
    echo -e "${RED}❌ OpenCV non installato${NC}"
    exit 1
}
echo "   ✅ OpenCV OK"

python3 -c "from cv_bridge import CvBridge" 2>/dev/null || {
    echo -e "${RED}❌ cv_bridge non installato${NC}"
    echo "   Installa: sudo apt install ros-humble-cv-bridge"
    exit 1
}
echo "   ✅ cv_bridge OK"

# ========================================
# 2. Setup ROS2 workspace
# ========================================

echo ""
echo -e "${YELLOW}[2/5] Setup ROS2 workspace...${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash
echo "   ✅ ROS2 sourced"

# Aggiungi script directory al PYTHONPATH
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
echo "   ✅ PYTHONPATH configurato"

# ========================================
# 3. Rendi eseguibili i nodi Python
# ========================================

echo ""
echo -e "${YELLOW}[3/5] Setup nodi Python...${NC}"

chmod +x "$SCRIPT_DIR/vision_yolo_detector.py"
chmod +x "$SCRIPT_DIR/moveit_vision_controller.py"
chmod +x "$SCRIPT_DIR/vision_robot_coordinator.py"
echo "   ✅ Nodi resi eseguibili"

# ========================================
# 4. Avvia camera Orbecc
# ========================================

echo ""
echo -e "${YELLOW}[4/5] Avvio camera Orbecc...${NC}"

# Verifica se camera già avviata
if ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
    echo "   ℹ️  Camera già avviata"
else
    echo "   Avvio driver camera..."
    
    # Avvia camera in background
    ros2 launch orbecc_camera gemini_330_series.launch.py > /tmp/orbecc_camera.log 2>&1 &
    CAMERA_PID=$!
    
    # Attendi avvio
    sleep 3
    
    # Verifica
    if ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
        echo "   ✅ Camera avviata (PID: $CAMERA_PID)"
    else
        echo -e "${RED}   ❌ Errore avvio camera${NC}"
        echo "   Log: /tmp/orbecc_camera.log"
        kill $CAMERA_PID 2>/dev/null || true
        exit 1
    fi
fi

# ========================================
# 5. Avvia sistema vision + robot
# ========================================

echo ""
echo -e "${YELLOW}[5/5] Avvio sistema vision...${NC}"

# Modalità: Launch file ROS2 o singoli nodi

if [ -f "$SCRIPT_DIR/launch_vision_robot_system.py" ]; then
    echo "   Usando launch file ROS2..."
    
    ros2 launch "$SCRIPT_DIR/launch_vision_robot_system.py" \
        robot_ip:=$ROBOT_IP \
        auto_mode:=$AUTO_MODE \
        yolo_model:=$YOLO_MODEL \
        confidence:=$CONFIDENCE \
        launch_camera:=false
else
    echo "   Avvio nodi singolarmente..."
    
    # Vision YOLO Detector
    echo "   → Vision YOLO Detector..."
    python3 "$SCRIPT_DIR/vision_yolo_detector.py" \
        --ros-args \
        -p yolo_model:=$YOLO_MODEL \
        -p confidence_threshold:=$CONFIDENCE \
        > /tmp/vision_detector.log 2>&1 &
    DETECTOR_PID=$!
    sleep 2
    
    # MoveIt Vision Controller
    echo "   → MoveIt Vision Controller..."
    python3 "$SCRIPT_DIR/moveit_vision_controller.py" \
        --ros-args \
        -p auto_mode:=$AUTO_MODE \
        > /tmp/moveit_controller.log 2>&1 &
    MOVEIT_PID=$!
    sleep 2
    
    # Vision Robot Coordinator
    echo "   → Vision Robot Coordinator..."
    python3 "$SCRIPT_DIR/vision_robot_coordinator.py" \
        --ros-args \
        -p robot_ip:=$ROBOT_IP \
        -p auto_pick:=$AUTO_MODE \
        > /tmp/coordinator.log 2>&1 &
    COORDINATOR_PID=$!
    sleep 2
    
    echo ""
    echo -e "${GREEN}✅ Sistema avviato!${NC}"
    echo ""
    echo "PIDs:"
    echo "  Camera:      $CAMERA_PID"
    echo "  Detector:    $DETECTOR_PID"
    echo "  MoveIt:      $MOVEIT_PID"
    echo "  Coordinator: $COORDINATOR_PID"
    echo ""
    echo "Logs:"
    echo "  Camera:      /tmp/orbecc_camera.log"
    echo "  Detector:    /tmp/vision_detector.log"
    echo "  MoveIt:      /tmp/moveit_controller.log"
    echo "  Coordinator: /tmp/coordinator.log"
fi

# ========================================
# Info finale
# ========================================

echo ""
echo "=========================================="
echo "SISTEMA VISION ROBOT AVVIATO"
echo "=========================================="
echo ""
echo "ROS2 Topics disponibili:"
echo "  /camera/color/image_raw      - RGB camera feed"
echo "  /camera/depth/image_raw      - Depth camera feed"
echo "  /vision/detections           - YOLO detections 2D"
echo "  /vision/detections_3d        - YOLO detections 3D"
echo "  /vision/annotated_image      - Immagine con annotazioni"
echo "  /vision/selected_target      - Target selezionato"
echo "  /vision/system_status        - Status sistema"
echo ""
echo "Comandi utili:"
echo "  # Visualizza detections"
echo "  ros2 topic echo /vision/detections_3d"
echo ""
echo "  # Visualizza immagine annotata (con RViz o rqt_image_view)"
echo "  ros2 run rqt_image_view rqt_image_view /vision/annotated_image"
echo ""
echo "  # Invia comando pick"
echo "  ros2 topic pub --once /vision/coordinator_command std_msgs/String '{data: \"{\\\"command\\\": \\\"pick_target\\\"}\"}" 
echo ""
echo "  # Stop robot"
echo "  ros2 topic pub --once /vision/coordinator_command std_msgs/String '{data: \"{\\\"command\\\": \\\"stop\\\"}\"}"
echo ""
echo "Per fermare tutto: Ctrl+C oppure ./ferma_vision_robot_system.sh"
echo "=========================================="
echo ""

# Mantieni script attivo (se launch file)
if [ -f "$SCRIPT_DIR/launch_vision_robot_system.py" ]; then
    wait
fi




