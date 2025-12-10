#!/bin/bash
# Script per setup completo AI Accelerator - RISOLVE TUTTI I PROBLEMI

set -e

echo "=========================================="
echo "SETUP COMPLETO AI ACCELERATOR"
echo "=========================================="
echo ""

# 1. Source ROS2
echo "1. Setup ROS2..."
source /opt/ros/humble/setup.bash
echo "   ✅ ROS2 Humble sourceato"

# 2. Verifica ur_rtde
echo ""
echo "2. Verifica ur_rtde..."
if python3 -c "import ur_rtde" 2>/dev/null; then
    echo "   ✅ ur_rtde installato"
else
    echo "   📦 Installo ur_rtde..."
    python3 -m pip install --user ur-rtde
    echo "   ✅ ur_rtde installato"
fi

# 3. Compila driver UR ROS2
echo ""
echo "3. Compilazione driver UR ROS2..."
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

if [ ! -d "install/ur_robot_driver" ]; then
    echo "   📦 Compilo driver..."
    colcon build --packages-select ur_robot_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "   ✅ Driver compilato"
else
    echo "   ✅ Driver già compilato"
fi

source install/setup.bash
echo "   ✅ Workspace sourceato"

# 4. Verifica pacchetti
echo ""
echo "4. Verifica pacchetti ROS2..."
if ros2 pkg list | grep -q ur_robot_driver; then
    echo "   ✅ ur_robot_driver disponibile"
else
    echo "   ❌ ur_robot_driver NON trovato"
    exit 1
fi

# 5. Test connettività robot
echo ""
echo "5. Test connettività robot..."
ROBOT_IP="192.168.10.194"
if ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile: $ROBOT_IP"
    
    # Test socket
    if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/30002" 2>/dev/null; then
        echo "   ✅ Socket robot (30002) aperto"
    else
        echo "   ⚠️  Socket robot (30002) non raggiungibile"
    fi
else
    echo "   ⚠️  Robot NON raggiungibile: $ROBOT_IP"
fi

# 6. Verifica componenti Python
echo ""
echo "6. Verifica componenti Python..."
python3 -c "import mujoco; print('   ✅ MuJoCo')" 2>/dev/null || echo "   ❌ MuJoCo"
python3 -c "import cv2; print('   ✅ OpenCV')" 2>/dev/null || echo "   ❌ OpenCV"
python3 -c "import ultralytics; print('   ✅ YOLOv8')" 2>/dev/null || echo "   ❌ YOLOv8"
python3 -c "import open3d; print('   ✅ Open3D')" 2>/dev/null || echo "   ❌ Open3D"

# 7. Verifica file progetto
echo ""
echo "7. Verifica file progetto..."
cd ~/MekoAiAccelerator
if [ -f "remote_ur_control/web_interface.py" ]; then
    echo "   ✅ Web Interface presente"
else
    echo "   ❌ Web Interface mancante"
fi

if [ -f "ros2_bridge_fixed.py" ]; then
    echo "   ✅ ROS2 Bridge presente"
else
    echo "   ❌ ROS2 Bridge mancante"
fi

echo ""
echo "=========================================="
echo "✅ SETUP COMPLETATO!"
echo "=========================================="
echo ""
echo "Per avviare il driver UR:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "      ur_type:=ur5e \\"
echo "      robot_ip:=192.168.10.194 \\"
echo "      launch_rviz:=false"
echo ""
echo "Per avviare web interface:"
echo "  cd ~/MekoAiAccelerator"
echo "  export UR_ROBOT_IP=192.168.10.194"
echo "  python3 -m remote_ur_control.web_interface"
echo ""





