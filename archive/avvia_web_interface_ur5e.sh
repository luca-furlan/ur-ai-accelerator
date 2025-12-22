#!/bin/bash
# Script per avviare web interface per UR5e

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Aggiungi path Python per ur_rtde
export PYTHONPATH=~/.local/lib/python3.10/site-packages:$PYTHONPATH

cd ~/MekoAiAccelerator

echo "=========================================="
echo "AVVIO WEB INTERFACE UR5e"
echo "=========================================="
echo ""
echo "Robot IP: $UR_ROBOT_IP"
echo "Web Interface: http://192.168.10.191:$WEB_PORT"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

python3 -m remote_ur_control.web_interface











