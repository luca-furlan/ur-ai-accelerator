#!/bin/bash
# Script per avviare driver UR ROS2 con controller NON-scaled (fix segmentation fault ARM64)

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=========================================="
echo "AVVIO DRIVER UR ROS2 - UR5e (FIXED)"
echo "=========================================="
echo ""
echo "Robot IP: 192.168.10.194"
echo "Robot Type: ur5e"
echo "Controller: joint_trajectory_controller (NON-scaled - fix ARM64)"
echo ""

# Verifica connettività
echo "Verifica connettività robot..."
if ping -c 1 -W 2 192.168.10.194 > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile"
else
    echo "❌ Robot NON raggiungibile!"
    exit 1
fi

echo ""
echo "IMPORTANTE: Sul Teach Pendant:"
echo "  1. Avvia programma con External Control"
echo "  2. IP Host: 192.168.10.191"
echo "  3. Porta: 50002"
echo "  4. Premi PLAY"
echo "  5. Verifica che programma sia in stato PLAYING"
echo ""
echo "Premi CTRL+C per fermare"
echo ""
echo "=========================================="
echo ""

# Usa joint_trajectory_controller invece di scaled_joint_trajectory_controller
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false \
    initial_joint_controller:=joint_trajectory_controller











