#!/bin/bash
# Verifica installazione driver ROS2 secondo documentazione ufficiale GitHub

set -e

echo "================================================================================"
echo "VERIFICA INSTALLAZIONE UNIVERSAL ROBOTS ROS2 DRIVER"
echo "Documentazione: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver"
echo "================================================================================"
echo

# Configurazione
ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

# PASSO 1: Verifica ROS2 Humble
echo "1. VERIFICA ROS2 HUMBLE"
echo "------------------------"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble installato"
    echo "   Versione: $(ros2 --version 2>/dev/null || echo 'N/A')"
else
    echo "❌ ROS2 Humble NON installato"
    exit 1
fi
echo

# PASSO 2: Verifica Driver installato via apt
echo "2. VERIFICA DRIVER INSTALLATO VIA APT"
echo "--------------------------------------"
if dpkg -l | grep -q "ros-humble-ur"; then
    echo "✅ Driver installato via apt"
    INSTALLED_PKGS=$(dpkg -l | grep "ros-humble-ur" | awk '{print $2}')
    echo "   Pacchetti:"
    echo "$INSTALLED_PKGS" | sed 's/^/     - /'
else
    echo "⚠️  Driver NON installato via apt"
    echo "   Prova: sudo apt-get install ros-humble-ur"
fi
echo

# PASSO 3: Verifica Driver da sorgente
echo "3. VERIFICA DRIVER DA SORGENTE"
echo "-------------------------------"
if [ -d ~/ros2_ws/src/Universal_Robots_ROS2_Driver ]; then
    echo "✅ Driver presente in workspace sorgente"
    echo "   Path: ~/ros2_ws/src/Universal_Robots_ROS2_Driver"
    
    if [ -d ~/ros2_ws/install/ur_robot_driver ]; then
        echo "✅ Driver compilato"
        if [ -f ~/ros2_ws/install/setup.bash ]; then
            source ~/ros2_ws/install/setup.bash
            echo "✅ Workspace configurato"
        fi
    else
        echo "⚠️  Driver NON compilato"
    fi
else
    echo "⚠️  Driver NON presente in workspace sorgente"
fi
echo

# PASSO 4: Verifica Driver disponibile in ROS2
echo "4. VERIFICA DRIVER DISPONIBILE IN ROS2"
echo "---------------------------------------"
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

if ros2 pkg list | grep -q "ur_robot_driver"; then
    echo "✅ ur_robot_driver disponibile"
    PKG_PATH=$(ros2 pkg prefix ur_robot_driver 2>/dev/null || echo "N/A")
    echo "   Path: $PKG_PATH"
    
    # Verifica launch file
    if ros2 pkg prefix ur_robot_driver >/dev/null 2>&1; then
        LAUNCH_FILE=$(ros2 pkg prefix ur_robot_driver)/share/ur_robot_driver/launch/ur_control.launch.py
        if [ -f "$LAUNCH_FILE" ]; then
            echo "✅ Launch file trovato: ur_control.launch.py"
        else
            echo "⚠️  Launch file NON trovato"
        fi
    fi
else
    echo "❌ ur_robot_driver NON disponibile"
    echo "   Installa con: sudo apt-get install ros-humble-ur"
    echo "   OPPURE compila da sorgente"
    exit 1
fi
echo

# PASSO 5: Verifica stato robot
echo "5. VERIFICA STATO ROBOT"
echo "------------------------"
python3 << 'PYTHON'
import socket
import sys

ROBOT_IP = "192.168.10.194"

try:
    # Test connessione Dashboard Server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    # Leggi robot mode
    sock.sendall(b"robotmode\n")
    robot_mode = sock.recv(1024).decode().strip()
    print(f"   Robot Mode: {robot_mode}")
    
    # Leggi program state
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    print(f"   Program State: {program_state}")
    
    # Leggi safety mode
    sock.sendall(b"safetymode\n")
    safety_mode = sock.recv(1024).decode().strip()
    print(f"   Safety Mode: {safety_mode}")
    
    sock.close()
    
    # Verifica condizioni
    if "RUNNING" not in robot_mode:
        print()
        print("   ⚠️  Robot NON in RUNNING mode")
        print("   💡 Accendi il robot e mettilo in RUNNING")
    elif "PLAYING" not in program_state:
        print()
        print("   ⚠️  Programma NON in PLAYING")
        print("   💡 Avvia programma External Control sul teach pendant")
    else:
        print()
        print("   ✅ Robot pronto per ROS2 control")
        
except socket.timeout:
    print("   ❌ Timeout connessione robot")
    print("   💡 Verifica IP: $ROBOT_IP")
    sys.exit(1)
except Exception as e:
    print(f"   ❌ Errore: {e}")
    sys.exit(1)
PYTHON

ROBOT_READY=$?
echo

# PASSO 6: Riepilogo
echo "================================================================================"
echo "RIEPILOGO"
echo "================================================================================"
echo

if [ $ROBOT_READY -eq 0 ]; then
    echo "✅ INSTALLAZIONE COMPLETA"
    echo
    echo "Prossimi passi secondo documentazione ufficiale:"
    echo
    echo "1. Assicurati che il programma External Control sia in PLAYING sul teach pendant"
    echo
    echo "2. Avvia il driver con:"
    echo "   ros2 launch ur_robot_driver ur_control.launch.py \\"
    echo "       ur_type:=${ROBOT_TYPE} \\"
    echo "       robot_ip:=${ROBOT_IP}"
    echo
    echo "3. Verifica controller attivi:"
    echo "   ros2 control list_controllers"
    echo
    echo "4. Test movimento (se disponibile):"
    echo "   ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"
    echo
else
    echo "⚠️  INSTALLAZIONE INCOMPLETA"
    echo
    echo "Risolvi i problemi sopra indicati prima di procedere"
fi









