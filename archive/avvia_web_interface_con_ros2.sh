#!/bin/bash
# Avvia Web Interface con ROS2 configurato correttamente

echo "=== AVVIO WEB INTERFACE CON ROS2 ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Verifica ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 non configurato"
    exit 1
fi

echo "✅ ROS2 configurato: $ROS_DISTRO"

# Verifica rclpy
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "❌ rclpy non disponibile"
    echo "💡 Installa: sudo apt install ros-humble-rclpy"
    exit 1
fi

echo "✅ rclpy disponibile"

# Kill processi esistenti
pkill -f "web_interface.py" 2>/dev/null
sleep 2

# Imposta variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8081

# Verifica porta
if lsof -Pi :$WEB_PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠️ Porta $WEB_PORT già in uso, kill processo..."
    fuser -k $WEB_PORT/tcp 2>/dev/null
    sleep 2
fi

echo "Avvio Web Interface..."
echo "URL: http://$(hostname -I | awk '{print $1}'):$WEB_PORT"
echo ""

cd ~/MekoAiAccelerator
python3 -m remote_ur_control.web_interface











