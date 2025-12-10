#!/bin/bash
# Script per avviare driver UR ROS2 per UR5e

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=========================================="
echo "AVVIO DRIVER UR ROS2 - UR5e"
echo "=========================================="
echo ""
echo "Robot IP: 192.168.10.194"
echo "Robot Type: ur5e"
echo ""
echo "IMPORTANTE: Sul Teach Pendant:"
echo "  1. Avvia programma con External Control"
echo "  2. IP Host: 192.168.10.191"
echo "  3. Porta: 50002"
echo "  4. Premi PLAY"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false





