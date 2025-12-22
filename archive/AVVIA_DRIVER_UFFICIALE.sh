#!/bin/bash
# Avvia driver ROS2 secondo documentazione UFFICIALE GitHub
# https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

set -e

echo "================================================================================"
echo "AVVIO UNIVERSAL ROBOTS ROS2 DRIVER"
echo "Documentazione: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

# Configura ROS2
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ ROS2 workspace configurato"
fi

# Verifica stato robot
echo "Verifica stato robot..."
python3 << 'PYTHON'
import socket
import sys

ROBOT_IP = "192.168.10.194"

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    sock.close()
    
    if "PLAYING" not in program_state:
        print(f"   ⚠️  Programma: {program_state}")
        print()
        print("   ❌ ERRORE: Programma NON in PLAYING!")
        print()
        print("   💡 AZIONE RICHIESTA:")
        print("      1. Vai sul teach pendant del robot")
        print("      2. Avvia il programma 'remote_control.urp' (o il tuo programma External Control)")
        print("      3. Premi PLAY sul teach pendant")
        print("      4. Verifica che 'Remote Control' sia attivo")
        print()
        print("   ⏸️  Attendo che avvii il programma...")
        print("   (Premi CTRL+C per annullare)")
        sys.exit(1)
    else:
        print(f"   ✅ Programma: {program_state}")
        
except Exception as e:
    print(f"   ❌ Errore connessione robot: {e}")
    sys.exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "================================================================================"
    echo "ATTENZIONE: Avvia il programma sul teach pendant prima di continuare!"
    echo "================================================================================"
    echo
    read -p "Premi INVIO quando il programma è in PLAYING sul teach pendant..."
    
    # Verifica di nuovo
    python3 << 'PYTHON'
import socket
import sys

ROBOT_IP = "192.168.10.194"

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    sock.close()
    
    if "PLAYING" not in program_state:
        print(f"   ❌ Programma ancora NON in PLAYING: {program_state}")
        sys.exit(1)
    else:
        print(f"   ✅ Programma in PLAYING: {program_state}")
        
except Exception as e:
    print(f"   ❌ Errore: {e}")
    sys.exit(1)
PYTHON

    if [ $? -ne 0 ]; then
        echo
        echo "❌ Programma ancora non in PLAYING. Avvia il programma sul teach pendant!"
        exit 1
    fi
fi

echo
echo "================================================================================"
echo "AVVIO DRIVER ROS2"
echo "================================================================================"
echo
echo "Comando ufficiale dalla documentazione GitHub:"
echo
echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "    ur_type:=${ROBOT_TYPE} \\"
echo "    robot_ip:=${ROBOT_IP}"
echo
echo "================================================================================"
echo

# Ferma eventuali driver già in esecuzione
echo "Ferma eventuali driver già in esecuzione..."
pkill -f "ur_control.launch.py" 2>/dev/null || true
sleep 2

# Avvia driver
echo "Avvio driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=${ROBOT_TYPE} \
    robot_ip:=${ROBOT_IP} \
    launch_rviz:=false









