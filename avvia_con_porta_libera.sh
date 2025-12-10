#!/bin/bash
# Script per avviare web interface su porta libera

echo "=========================================="
echo "AVVIO WEB INTERFACE - PORTA LIBERA"
echo "=========================================="
echo ""

# Ferma processi esistenti
echo "1. Fermo processi esistenti..."
pkill -f web_interface 2>/dev/null || true
sleep 2

# Libera porta 8080
echo "2. Libero porta 8080..."
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

# Prova porta 8081 se 8080 occupata
PORT=8080
if lsof -i :8080 2>/dev/null | grep -q LISTEN; then
    echo "⚠️  Porta 8080 ancora occupata, uso 8081..."
    PORT=8081
fi

# Vai nella directory
cd ~/MekoAiAccelerator

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Configura
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=$PORT

echo ""
echo "3. Avvio web interface su porta $PORT..."
echo ""

# Avvia
python3 -m remote_ur_control.web_interface

