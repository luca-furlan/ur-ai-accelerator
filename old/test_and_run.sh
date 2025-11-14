#!/bin/bash
# Script completo di test e avvio sull'AI Accelerator

set -e

echo "=========================================="
echo "TEST E AVVIO WEB INTERFACE"
echo "=========================================="

cd ~/MekoAiAccelerator

# 1. Verifica file
echo ""
echo "1. Verifica file..."
if [ ! -f remote_ur_control/web_interface.py ]; then
    echo "❌ File non trovato! Estrai deploy.zip prima"
    exit 1
fi
echo "✅ File trovati"

# 2. Installa Flask se necessario
echo ""
echo "2. Verifica Flask..."
python3 -c "import flask" 2>/dev/null || {
    echo "Installing Flask..."
    python3 -m pip install --user flask
}
echo "✅ Flask OK"

# 3. Test ROS2
echo ""
echo "3. Test ROS2..."
ROS2_AVAILABLE=false
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    python3 -c "import rclpy" 2>/dev/null && {
        ROS2_AVAILABLE=true
        echo "✅ ROS2 disponibile!"
        
        # Verifica driver UR
        if [ -d ~/ros2_ws/install ]; then
            source ~/ros2_ws/install/setup.bash 2>/dev/null
            ros2 pkg list | grep ur_robot_driver > /dev/null && {
                echo "✅ UR Robot Driver installato"
            } || {
                echo "⚠️ UR Robot Driver non trovato"
            }
        fi
    } || {
        echo "⚠️ ROS2 installato ma rclpy non disponibile"
        echo "   La web interface userà fallback socket"
    }
else
    echo "⚠️ ROS2 non installato - userà fallback socket"
fi

# 4. Test import
echo ""
echo "4. Test import..."
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
except ImportError as e:
    print(f"⚠️ ROS2Bridge non disponibile: {e}")
PYTHON_EOF

# 5. Test connessione robot
echo ""
echo "5. Test connessione robot..."
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

# 6. Avvia server
echo ""
echo "=========================================="
echo "AVVIO WEB INTERFACE"
echo "=========================================="
echo ""
echo "Server in avvio..."
echo "Apri: http://192.168.10.191:8080"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Source ROS2 se disponibile per il bridge
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    [ -d ~/ros2_ws/install ] && source ~/ros2_ws/install/setup.bash 2>/dev/null || true
fi

python3 -m remote_ur_control.web_interface


