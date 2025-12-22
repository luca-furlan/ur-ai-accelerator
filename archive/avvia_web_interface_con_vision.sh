#!/bin/bash

# ================================================================================
# AVVIO WEB INTERFACE + VISION SYSTEM
# Estende la web interface esistente aggiungendo detection YOLOv8
# ================================================================================

set +e

ROBOT_IP="${UR_ROBOT_IP:-192.168.10.194}"
WEB_PORT="${WEB_PORT:-8080}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"

echo "=================================================================================="
echo "🤖 AVVIO WEB INTERFACE + VISION SYSTEM"
echo "=================================================================================="
echo ""
echo "Configurazione:"
echo "  Robot IP: $ROBOT_IP"
echo "  Web Interface: http://$WEB_HOST:$WEB_PORT"
echo "  Vision System: Camera Orbecc + YOLOv8"
echo ""

# ========================================
# 1. Source ROS2
# ========================================
echo "[1/5] Configurazione ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✅ ROS2 Humble"
else
    echo "  ❌ ROS2 Humble non trovato"
    exit 1
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "  ✅ ROS2 Workspace"
fi

export PYTHONPATH=$(pwd):$PYTHONPATH

# ========================================
# 2. Verifica dipendenze vision
# ========================================
echo ""
echo "[2/5] Verifica dipendenze vision..."

# YOLOv8
if python3 -c "from ultralytics import YOLO" 2>/dev/null; then
    echo "  ✅ YOLOv8 installato"
else
    echo "  ⚠️  YOLOv8 non installato (installazione...)"
    pip3 install ultralytics --user -q
fi

# OpenCV
if python3 -c "import cv2" 2>/dev/null; then
    echo "  ✅ OpenCV installato"
else
    echo "  ⚠️  OpenCV mancante"
fi

# cv_bridge
if python3 -c "from cv_bridge import CvBridge" 2>/dev/null; then
    echo "  ✅ cv_bridge disponibile"
else
    echo "  ⚠️  cv_bridge mancante (installa: sudo apt install ros-humble-cv-bridge)"
fi

# ========================================
# 3. Verifica e kill processi esistenti
# ========================================
echo ""
echo "[3/5] Pulizia processi esistenti..."

CURRENT_PID=$$

# Kill web interface esistente
EXISTING_WEB=$(pgrep -f "python.*web_interface" 2>/dev/null || true)
if [ -n "$EXISTING_WEB" ]; then
    for pid in $EXISTING_WEB; do
        if [ "$pid" != "$CURRENT_PID" ] && [ "$pid" != "$PPID" ]; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    echo "  ✅ Web interface terminata"
    sleep 1
fi

# Kill vision nodes esistenti
pkill -f "vision_yolo_detector" 2>/dev/null || true
pkill -f "moveit_vision_controller" 2>/dev/null || true
pkill -f "vision_robot_coordinator" 2>/dev/null || true
echo "  ✅ Vision nodes terminati"

# ========================================
# 4. Integra Vision API in web interface
# ========================================
echo ""
echo "[4/5] Setup Vision API..."

if [ -f "integra_vision_web_interface.py" ]; then
    # Verifica se già integrato
    if grep -q "vision_web_api" remote_ur_control/web_interface.py 2>/dev/null; then
        echo "  ✅ Vision API già integrata"
    else
        echo "  → Integrazione Vision API..."
        python3 integra_vision_web_interface.py
        echo "  ✅ Vision API integrata"
    fi
else
    echo "  ⚠️  Script integrazione non trovato (skip)"
fi

# ========================================
# 5. Avvia sistema completo
# ========================================
echo ""
echo "[5/5] Avvio componenti..."
echo ""

# Flag per cleanup
USER_INTERRUPTED=0

cleanup() {
    if [ $USER_INTERRUPTED -eq 1 ]; then
        exit 0
    fi
    USER_INTERRUPTED=1
    
    echo ""
    echo "🛑 Fermando sistema..."
    
    # Kill web interface
    for pid in $(pgrep -f "python.*web_interface" 2>/dev/null || true); do
        if [ "$pid" != "$CURRENT_PID" ] && [ "$pid" != "$PPID" ]; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    
    # Kill vision nodes
    pkill -f "vision_yolo_detector" 2>/dev/null || true
    pkill -f "moveit_vision_controller" 2>/dev/null || true
    pkill -f "vision_robot_coordinator" 2>/dev/null || true
    
    sleep 1
    
    # Force kill se necessario
    pkill -9 -f "vision_yolo_detector" 2>/dev/null || true
    pkill -9 -f "moveit_vision_controller" 2>/dev/null || true
    pkill -9 -f "vision_robot_coordinator" 2>/dev/null || true
    
    echo "✅ Sistema fermato"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Avvia camera Orbecc (se non già attiva)
echo "→ Verifica camera Orbecc..."
if ! ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
    echo "  → Avvio driver camera..."
    ros2 launch orbecc_camera gemini_330_series.launch.py > /tmp/orbecc_camera.log 2>&1 &
    CAMERA_PID=$!
    sleep 3
    
    if ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
        echo "  ✅ Camera avviata (PID: $CAMERA_PID)"
    else
        echo "  ⚠️  Camera non disponibile (continuo comunque)"
    fi
else
    echo "  ✅ Camera già attiva"
fi

# Avvia Vision YOLO Detector
if [ -f "vision_yolo_detector.py" ]; then
    echo "→ Avvio Vision YOLO Detector..."
    python3 vision_yolo_detector.py > /tmp/vision_detector.log 2>&1 &
    DETECTOR_PID=$!
    echo "  ✅ Detector avviato (PID: $DETECTOR_PID, log: /tmp/vision_detector.log)"
    sleep 2
else
    echo "  ⚠️  vision_yolo_detector.py non trovato (skip)"
fi

# Avvia MoveIt Vision Controller
if [ -f "moveit_vision_controller.py" ]; then
    echo "→ Avvio MoveIt Vision Controller..."
    python3 moveit_vision_controller.py > /tmp/moveit_controller.log 2>&1 &
    MOVEIT_PID=$!
    echo "  ✅ Controller avviato (PID: $MOVEIT_PID, log: /tmp/moveit_controller.log)"
    sleep 1
else
    echo "  ⚠️  moveit_vision_controller.py non trovato (skip)"
fi

# Avvia Vision Robot Coordinator
if [ -f "vision_robot_coordinator.py" ]; then
    echo "→ Avvio Vision Robot Coordinator..."
    python3 vision_robot_coordinator.py \
        --ros-args -p robot_ip:=$ROBOT_IP \
        > /tmp/coordinator.log 2>&1 &
    COORDINATOR_PID=$!
    echo "  ✅ Coordinator avviato (PID: $COORDINATOR_PID, log: /tmp/coordinator.log)"
    sleep 2
else
    echo "  ⚠️  vision_robot_coordinator.py non trovato (skip)"
fi

# Esporta variabili
export UR_ROBOT_IP="$ROBOT_IP"
export WEB_HOST="$WEB_HOST"
export WEB_PORT="$WEB_PORT"
export WEB_DEBUG="0"

# ========================================
# Info finale
# ========================================
echo ""
echo "=================================================================================="
echo "✅ SISTEMA ONLINE!"
echo "=================================================================================="
echo ""
echo "Accedi da browser:"
echo "  http://$(hostname -I | awk '{print $1}'):$WEB_PORT"
echo ""
echo "Componenti attivi:"
if [ -n "$CAMERA_PID" ]; then
    echo "  📷 Camera Orbecc (PID: $CAMERA_PID)"
fi
if [ -n "$DETECTOR_PID" ]; then
    echo "  👁️  Vision YOLO Detector (PID: $DETECTOR_PID)"
fi
if [ -n "$MOVEIT_PID" ]; then
    echo "  🎯 MoveIt Controller (PID: $MOVEIT_PID)"
fi
if [ -n "$COORDINATOR_PID" ]; then
    echo "  🤖 Vision Coordinator (PID: $COORDINATOR_PID)"
fi
echo ""
echo "Nuove funzionalità disponibili:"
echo "  • GET  /api/vision/status - Status vision system"
echo "  • GET  /api/vision/detections - Detections real-time"
echo "  • POST /api/vision/pick_target - Pick oggetto rilevato"
echo "  • POST /api/vision/select_class - Seleziona classe target"
echo ""
echo "Log:"
echo "  Camera:     /tmp/orbecc_camera.log"
echo "  Detector:   /tmp/vision_detector.log"
echo "  Controller: /tmp/moveit_controller.log"
echo "  Coordinator: /tmp/coordinator.log"
echo ""
echo "Premi CTRL+C per fermare tutto"
echo "=================================================================================="
echo ""

# ========================================
# Avvia web interface (foreground)
# ========================================
echo "🚀 Avvio web interface..."
echo ""

cd ~/MekoAiAccelerator
python3 -m remote_ur_control.web_interface

# Se arriviamo qui, web interface è terminata
cleanup




