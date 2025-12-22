#!/bin/bash
# Script per avviare tutto il sistema

echo "=========================================="
echo "AVVIO SISTEMA COMPLETO"
echo "=========================================="
echo ""

# 1. Avvia driver UR ROS2 in background
echo "1. Avvio driver UR ROS2..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Verifica se già in esecuzione
if ros2 node list 2>/dev/null | grep -q ur; then
    echo "   ✅ Driver UR già in esecuzione"
else
    echo "   Avvio driver UR in background..."
    nohup ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false > /tmp/ur_driver.log 2>&1 &
    sleep 5
    if ros2 node list 2>/dev/null | grep -q ur; then
        echo "   ✅ Driver UR avviato"
    else
        echo "   ⚠️  Driver UR potrebbe non essere avviato - controlla /tmp/ur_driver.log"
    fi
fi

# 2. Avvia web interface
echo ""
echo "2. Avvio web interface..."
pkill -f web_interface 2>/dev/null || true
sleep 2

cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8081

echo "   Avvio web interface su porta 8081..."
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
sleep 3

if pgrep -f web_interface > /dev/null; then
    echo "   ✅ Web interface avviata"
else
    echo "   ❌ Errore avvio web interface - controlla /tmp/web_interface.log"
fi

# 3. Verifica stato
echo ""
echo "3. Verifica stato..."
echo ""
echo "   Driver UR ROS2:"
ros2 node list 2>/dev/null | grep ur || echo "      ❌ Non trovato"

echo ""
echo "   Topic ROS2:"
if ros2 topic list 2>/dev/null | grep -q forward_velocity; then
    echo "      ✅ /forward_velocity_controller/commands presente"
else
    echo "      ❌ Topic non presente (attendi qualche secondo)"
fi

echo ""
echo "   Web interface:"
if pgrep -f web_interface > /dev/null; then
    echo "      ✅ In esecuzione (PID: $(pgrep -f web_interface))"
else
    echo "      ❌ Non in esecuzione"
fi

echo ""
echo "=========================================="
echo "✅ SISTEMA AVVIATO!"
echo "=========================================="
echo ""
echo "🌐 Web interface: http://192.168.10.191:8081"
echo ""
echo "⚠️  IMPORTANTE: Sul teach pendant del robot:"
echo "   1. Avvia programma con External Control"
echo "   2. IP Host: 192.168.10.191"
echo "   3. Porta: 50002"
echo "   4. Premi PLAY"
echo ""
echo "📋 Log driver UR: tail -f /tmp/ur_driver.log"
echo "📋 Log web interface: tail -f /tmp/web_interface.log"
echo ""

