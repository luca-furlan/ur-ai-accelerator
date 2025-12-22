#!/bin/bash
# Script per avviare driver UR ROS2 secondo documentazione ufficiale

set -e

echo "================================================================================"
echo "AVVIO DRIVER UR ROS2 - DOCUMENTAZIONE UFFICIALE"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

# Source ROS2
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ ROS2 workspace configurato"
else
    echo "❌ Workspace ROS2 non trovato!"
    exit 1
fi

# Verifica che il robot sia raggiungibile
echo "Verifica connessione robot..."
if ! ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "❌ Robot non raggiungibile: $ROBOT_IP"
    exit 1
fi
echo "✅ Robot raggiungibile"

# Verifica che il programma sia in PLAYING
echo "Verifica stato robot..."
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    sock.close()
    
    if "PLAYING" not in program_state:
        print(f"⚠️  ATTENZIONE: Programma NON in PLAYING: {program_state}")
        print("💡 Configura il robot con External Control URCap:")
        print("   1. Installa External Control URCap")
        print("   2. Crea programma con External Control")
        print("   3. IP Host: 192.168.10.191, Porta: 50002")
        print("   4. Avvia programma in PLAYING")
        exit(1)
    else:
        print(f"✅ Programma in PLAYING: {program_state}")
except Exception as e:
    print(f"❌ Errore: {e}")
    exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "Configura prima il robot con External Control URCap!"
    exit 1
fi

echo
echo "================================================================================"
echo "AVVIO DRIVER UR ROS2"
echo "================================================================================"
echo
echo "Robot IP: $ROBOT_IP"
echo "Robot Type: $ROBOT_TYPE"
echo "AI Accelerator IP: 192.168.10.191"
echo
echo "Aspetta che vedi:"
echo "  [INFO] [ur_robot_driver]: Robot connected"
echo "  [INFO] [ur_robot_driver]: Controllers started"
echo
echo "Premi CTRL+C per fermare"
echo
echo "================================================================================"
echo

# Avvia driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=${ROBOT_TYPE} \
    robot_ip:=${ROBOT_IP} \
    launch_rviz:=false










