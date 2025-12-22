#!/bin/bash

# ========================================
# Ferma Sistema Vision + Robot
# ========================================

echo "Fermando sistema Vision + Robot..."

# Trova e uccidi processi Python relativi
pkill -f "vision_yolo_detector.py"
pkill -f "moveit_vision_controller.py"
pkill -f "vision_robot_coordinator.py"
pkill -f "launch_vision_robot_system.py"

# Ferma camera Orbecc
pkill -f "orbecc_camera"

# Ferma eventuali nodi ROS2
ros2 node list 2>/dev/null | grep -E "(vision|camera)" | while read node; do
    echo "Killing node: $node"
    ros2 lifecycle set $node shutdown 2>/dev/null || true
done

echo "✅ Sistema fermato"




