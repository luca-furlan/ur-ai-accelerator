#!/bin/bash
# Start ROS2 UR control with web interface

set -e

echo "🚀 Starting UR ROS2 Control System"
echo "=================================="

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start rosbridge in background
echo "📡 Starting rosbridge WebSocket server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
sleep 2

# Start UR driver
echo "🤖 Starting UR robot driver..."
echo "   Robot IP: 192.168.10.194"
echo "   Robot Type: ur5e"
echo ""
echo "⚠️  IMPORTANT: On the Teach Pendant, you must:"
echo "   1. Have External Control URCap installed"
echo "   2. Load program with External Control node (Host: 192.168.10.191, Port: 50002)"
echo "   3. Press PLAY"
echo ""
echo "Press Ctrl+C to stop everything"
echo ""

# Trap Ctrl+C to cleanup
trap "echo ''; echo '🛑 Stopping...'; kill $ROSBRIDGE_PID 2>/dev/null; exit 0" INT TERM

# Start UR driver (this blocks)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false \
    headless_mode:=true

