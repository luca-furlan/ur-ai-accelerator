#!/bin/bash
# Script per avvio rapido (se tutto è già configurato)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

echo "================================================================================"
echo "AVVIO RAPIDO DRIVER ROS2"
echo "================================================================================"
echo

# Verifica porta 50002 (solo avviso, non bloccante)
echo "Verifica porta 50002..."
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ROBOT_IP, 50002))
    sock.close()
    
    if result != 0:
        print("   ⚠️  Porta 50002 CHIUSA (normale se programma non è ancora in PLAYING)")
        print("   💡 Avvia il programma External Control sul Teach Pendant DOPO che il driver è avviato")
    else:
        print("   ✅ Porta 50002 aperta")
except Exception as e:
    print(f"   ⚠️  Non riesco a verificare porta: {e}")
PYTHON

echo
echo "⚠️  IMPORTANTE: Avvia il programma External Control sul Teach Pendant"
echo "   DOPO che vedi 'Waiting for connection...' nel driver ROS2"
echo

# Configura ROS2
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Ferma eventuali driver già in esecuzione
pkill -f "ur_control.launch.py" 2>/dev/null || true
sleep 2

# Avvia driver
echo
echo "Avvio driver ROS2..."
echo "Premi CTRL+C per fermare"
echo

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=${ROBOT_TYPE} \
    robot_ip:=${ROBOT_IP} \
    launch_rviz:=false


