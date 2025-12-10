#!/bin/bash
# Script per fermare tutti i processi ROS2 e driver

set -e

echo "================================================================================"
echo "FERMA TUTTI I PROCESSI ROS2"
echo "================================================================================"
echo

# Ferma driver ROS2
echo "1. Ferma driver ROS2..."
pkill -f "ur_control.launch.py" 2>/dev/null && echo "   ✅ Driver ROS2 fermato" || echo "   ℹ️  Nessun driver ROS2 in esecuzione"

# Ferma test movimento
echo "2. Ferma test movimento..."
pkill -f "test_scaled_joint_trajectory_controller" 2>/dev/null && echo "   ✅ Test movimento fermato" || echo "   ℹ️  Nessun test in esecuzione"

# Ferma altri processi ROS2
echo "3. Ferma altri processi ROS2..."
pkill -f "ros2 launch" 2>/dev/null && echo "   ✅ Altri processi ROS2 fermati" || echo "   ℹ️  Nessun altro processo ROS2"

sleep 1

echo
echo "================================================================================"
echo "VERIFICA PROCESSI FERMATI"
echo "================================================================================"
echo

if pgrep -f "ur_robot_driver\|ur_ros2_control\|ros2 launch" > /dev/null; then
    echo "⚠️  Alcuni processi sono ancora in esecuzione:"
    pgrep -f "ur_robot_driver\|ur_ros2_control\|ros2 launch" | xargs ps -p
else
    echo "✅ Tutti i processi ROS2 sono stati fermati"
fi

echo



