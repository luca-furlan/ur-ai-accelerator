#!/bin/bash
# Script completo per testare ROS2 e web interface

set -e

echo "=========================================="
echo "TEST COMPLETO ROS2 + WEB INTERFACE"
echo "=========================================="

cd ~/MekoAiAccelerator

# 1. Verifica ROS2
echo ""
echo "1. Verifica ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    if python3 -c "import rclpy" 2>/dev/null; then
        echo "✅ ROS2 rclpy disponibile"
    else
        echo "⚠️ ROS2 installato ma rclpy non disponibile"
        echo "   Installazione rclpy..."
        pip3 install --user rclpy || echo "   Fallito - continua comunque"
    fi
else
    echo "❌ ROS2 non installato"
fi

# 2. Verifica driver UR
echo ""
echo "2. Verifica UR Driver..."
if [ -d ~/ros2_ws/install ]; then
    source ~/ros2_ws/install/setup.bash 2>/dev/null || true
    if ros2 pkg list | grep ur_robot_driver > /dev/null 2>&1; then
        echo "✅ UR Robot Driver installato"
    else
        echo "⚠️ UR Robot Driver non trovato"
    fi
else
    echo "⚠️ Workspace ROS2 non trovato"
fi

# 3. Test import Python
echo ""
echo "3. Test import..."
python3 << 'PYTHON_EOF'
import sys
sys.path.insert(0, '.')
try:
    from remote_ur_control import web_interface
    print("✅ Import web_interface OK")
except Exception as e:
    print(f"❌ Errore import web_interface: {e}")
    sys.exit(1)

try:
    from ros2_bridge_fixed import ROS2Bridge
    print("✅ Import ROS2Bridge OK")
    
    # Test inizializzazione
    bridge = ROS2Bridge()
    if bridge.ensure_ros():
        print("✅ ROS2Bridge inizializzato correttamente")
    else:
        print("⚠️ ROS2Bridge non inizializzato (ROS2 non disponibile)")
except ImportError as e:
    print(f"⚠️ ROS2Bridge non disponibile: {e}")
except Exception as e:
    print(f"⚠️ Errore ROS2Bridge: {e}")
PYTHON_EOF

# 4. Test connessione robot
echo ""
echo "4. Test connessione robot..."
python3 << 'PYTHON_EOF'
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex(('192.168.10.194', 30002))
    sock.close()
    if result == 0:
        print("✅ Robot raggiungibile (porta 30002)")
    else:
        print("❌ Robot NON raggiungibile")
except Exception as e:
    print(f"❌ Errore: {e}")
PYTHON_EOF

# 5. Avvia server
echo ""
echo "=========================================="
echo "AVVIO WEB INTERFACE"
echo "=========================================="
echo ""
echo "Server in avvio su http://192.168.10.191:8080"
echo "Premi CTRL+C per fermare"
echo ""

export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Source ROS2 se disponibile
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    [ -d ~/ros2_ws/install ] && source ~/ros2_ws/install/setup.bash 2>/dev/null || true
fi

python3 -m remote_ur_control.web_interface


