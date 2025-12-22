#!/bin/bash
# Avvia ROS2 Bridge per Web Interface

echo "=== AVVIO ROS2 BRIDGE ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Verifica rclpy
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "❌ rclpy non disponibile"
    echo "💡 Installa: sudo apt install ros-humble-rclpy"
    exit 1
fi

# Kill processi esistenti
pkill -f "ros2_bridge_fixed.py" 2>/dev/null
sleep 1

# Avvia ROS2 Bridge
cd ~/MekoAiAccelerator
echo "Avvio ROS2 Bridge..."
nohup python3 ros2_bridge_fixed.py > /tmp/ros2_bridge.log 2>&1 &
BRIDGE_PID=$!

sleep 3

if ps -p $BRIDGE_PID > /dev/null; then
    echo "✅ ROS2 Bridge avviato (PID: $BRIDGE_PID)"
    echo "📋 Log: /tmp/ros2_bridge.log"
    echo ""
    echo "Verifica ultimi log:"
    tail -10 /tmp/ros2_bridge.log
else
    echo "❌ ROS2 Bridge non si è avviato"
    echo "📋 Controlla log:"
    cat /tmp/ros2_bridge.log
    exit 1
fi











