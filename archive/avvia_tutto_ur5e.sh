#!/bin/bash
# Script per avviare TUTTO per controllo UR5e

set -e

echo "=========================================="
echo "AVVIO SISTEMA COMPLETO UR5e"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Variabili
ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"
WEB_PORT="8080"

echo "1. Verifica connettività robot..."
if ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile"
else
    echo "   ❌ Robot NON raggiungibile!"
    exit 1
fi

echo ""
echo "2. Test connessione RTDE..."
cd ~/MekoAiAccelerator
python3 << 'PYTHON_EOF'
import sys
try:
    import ur_rtde
    print("   ✅ ur_rtde importabile")
    
    # Test connessione
    try:
        rtde = ur_rtde.RTDE("192.168.10.194", 30004)
        rtde.connect()
        state = rtde.receive()
        if state:
            print("   ✅ RTDE connesso al robot!")
            rtde.disconnect()
        else:
            print("   ⚠️  RTDE connesso ma nessun dato")
    except Exception as e:
        print(f"   ⚠️  RTDE non connesso: {e}")
except ImportError as e:
    print(f"   ❌ ur_rtde non importabile: {e}")
    sys.exit(1)
PYTHON_EOF

echo ""
echo "3. Verifica ROS2 driver..."
if ros2 pkg list | grep -q ur_robot_driver; then
    echo "   ✅ Driver UR ROS2 disponibile"
else
    echo "   ❌ Driver UR ROS2 NON disponibile"
    exit 1
fi

echo ""
echo "=========================================="
echo "SISTEMA PRONTO!"
echo "=========================================="
echo ""
echo "Per avviare il driver UR ROS2 (Terminale 1):"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "      ur_type:=ur5e \\"
echo "      robot_ip:=192.168.10.194 \\"
echo "      launch_rviz:=false"
echo ""
echo "Sul Teach Pendant:"
echo "  1. Avvia programma con External Control"
echo "  2. IP Host: 192.168.10.191"
echo "  3. Porta: 50002"
echo "  4. Premi PLAY"
echo ""
echo "Per avviare web interface (Terminale 2):"
echo "  cd ~/MekoAiAccelerator"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  export UR_ROBOT_IP=192.168.10.194"
echo "  export WEB_HOST=0.0.0.0"
echo "  export WEB_PORT=8080"
echo "  python3 -m remote_ur_control.web_interface"
echo ""
echo "Accedi da browser:"
echo "  http://192.168.10.191:8080"
echo ""











