#!/bin/bash

# ========================================
# DEPLOY COMPLETO VISION SYSTEM SU AI ACCELERATOR
# ========================================

set -e

AI_ACCELERATOR_IP="${AI_ACCELERATOR_IP:-192.168.1.100}"  # Modifica con IP reale
AI_ACCELERATOR_USER="${AI_ACCELERATOR_USER:-user}"
AI_ACCELERATOR_PATH="${AI_ACCELERATOR_PATH:-~/MekoAiAccelerator}"

echo "=========================================="
echo "DEPLOY VISION SYSTEM → AI ACCELERATOR"
echo "=========================================="
echo ""
echo "Target: $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH"
echo ""

# ========================================
# 1. Crea directory remota
# ========================================
echo "[1/5] Creazione directory remota..."
ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "mkdir -p $AI_ACCELERATOR_PATH"

# ========================================
# 2. Trasferisci file vision system
# ========================================
echo "[2/5] Trasferimento file vision system..."

# Nodi ROS2
scp vision_yolo_detector.py \
    moveit_vision_controller.py \
    vision_robot_coordinator.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Launch file
scp launch_vision_robot_system.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Script avvio/stop
scp avvia_vision_robot_system.sh \
    ferma_vision_robot_system.sh \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Test
scp test_vision_system.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Documentazione
scp README_VISION_ROBOT.md \
    QUICK_START_VISION.md \
    VISION_SYSTEM_COMPLETE.md \
    SISTEMA_VISION_INTEGRATO.txt \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Directory remote_ur_control (necessaria per imports)
scp -r remote_ur_control \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

echo "✅ File trasferiti"

# ========================================
# 3. Rendi eseguibili gli script
# ========================================
echo "[3/5] Configurazione permessi..."
ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "cd $AI_ACCELERATOR_PATH && chmod +x *.sh *.py"
echo "✅ Permessi configurati"

# ========================================
# 4. Installa dipendenze su AI Accelerator
# ========================================
echo "[4/5] Installazione dipendenze su AI Accelerator..."

ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP << 'ENDSSH'
cd ~/MekoAiAccelerator

echo "→ Source ROS2..."
source /opt/ros/humble/setup.bash

echo "→ Installazione Python packages..."
pip3 install ultralytics opencv-python numpy --user

echo "→ Verifica ROS2 packages..."
if ! ros2 pkg list | grep -q cv_bridge; then
    echo "⚠️  cv_bridge non trovato, installa: sudo apt install ros-humble-cv-bridge"
fi

if ! ros2 pkg list | grep -q vision_msgs; then
    echo "⚠️  vision_msgs non trovato, installa: sudo apt install ros-humble-vision-msgs"
fi

echo "→ Verifica camera Orbecc driver..."
if ! ros2 pkg list | grep -q orbbec_camera; then
    echo "⚠️  orbbec_camera non installato"
    echo "   Installa manualmente:"
    echo "   cd ~/ros2_ws/src"
    echo "   git clone https://github.com/orbbec/OrbbecSDK_ROS2.git"
    echo "   cd ~/ros2_ws && colcon build --packages-select orbbec_camera"
fi

echo "✅ Dipendenze verificate"
ENDSSH

echo "✅ Dipendenze installate"

# ========================================
# 5. Test sistema remoto
# ========================================
echo "[5/5] Test sistema su AI Accelerator..."

ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP << 'ENDSSH'
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
python3 test_vision_system.py
ENDSSH

echo ""
echo "=========================================="
echo "✅ DEPLOY COMPLETATO"
echo "=========================================="
echo ""
echo "Sistema vision deployato su AI Accelerator!"
echo ""
echo "Per avviare il sistema:"
echo "  ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo "  cd $AI_ACCELERATOR_PATH"
echo "  bash avvia_vision_robot_system.sh"
echo ""
echo "Dalla tua macchina Windows:"
echo "  python avvia_web_interface_locale.py"
echo ""
echo "=========================================="




