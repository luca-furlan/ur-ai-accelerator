#!/bin/bash
# Script per avviare web interface con ROS2 configurato

set -e  # Exit on error

cd ~/MekoAiAccelerator || exit 1

# Source ROS2 - CRITICAL: must be done before Python starts
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# CRITICAL: Ensure LD_LIBRARY_PATH includes ROS2 lib directory
# This is required for rclpy to find librcl_action.so and other shared libraries
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}

# Also ensure PYTHONPATH is set correctly
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}

# Export variabili
export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}
export WEB_HOST=${WEB_HOST:-0.0.0.0}
export WEB_PORT=${WEB_PORT:-8080}

# Verifica ROS2
echo "🔍 Verifica ROS2..."
echo "   LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
echo "   PYTHONPATH=${PYTHONPATH}"
python3 -c "import rclpy; print('✅ rclpy disponibile')" || {
    echo "⚠️ rclpy non disponibile - verificare LD_LIBRARY_PATH"
    exit 1
}

# Avvia web interface
echo "🌐 Avvio web interface su http://${WEB_HOST}:${WEB_PORT}..."
python3 -m remote_ur_control.web_interface

