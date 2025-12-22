#!/bin/bash
# Setup completo seguendo documentazione UFFICIALE GitHub
# https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

set -e

echo "================================================================================"
echo "SETUP UFFICIALE UNIVERSAL ROBOTS ROS2 DRIVER"
echo "Documentazione: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

# PASSO 1: Verifica/Installa Driver
echo "PASSO 1: INSTALLAZIONE DRIVER"
echo "-------------------------------"
echo

# Verifica se installato via apt
if dpkg -l | grep -q "ros-humble-ur"; then
    echo "✅ Driver installato via apt (ros-humble-ur)"
else
    echo "📦 Driver NON installato via apt"
    echo "💡 Installa con: sudo apt-get install ros-humble-ur"
    echo "   OPPURE compila da sorgente (vedi documentazione GitHub)"
    echo
    read -p "Vuoi installare ora? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get update
        sudo apt-get install -y ros-humble-ur
    else
        echo "⚠️  Continua con installazione da sorgente se già presente"
    fi
fi
echo

# Verifica workspace sorgente
echo "Verifica workspace sorgente..."
if [ -d ~/ros2_ws/src/Universal_Robots_ROS2_Driver ]; then
    echo "✅ Driver presente in workspace sorgente"
    echo "   Path: ~/ros2_ws/src/Universal_Robots_ROS2_Driver"
    
    # Verifica se compilato
    if [ -d ~/ros2_ws/install/ur_robot_driver ]; then
        echo "✅ Driver compilato"
    else
        echo "⚠️  Driver NON compilato"
        echo "💡 Compila con:"
        echo "   cd ~/ros2_ws"
        echo "   source /opt/ros/humble/setup.bash"
        echo "   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
    fi
else
    echo "⚠️  Driver NON presente in workspace sorgente"
fi
echo

# PASSO 2: Verifica ROS2
echo "PASSO 2: VERIFICA ROS2"
echo "----------------------"
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ ROS2 workspace configurato"
else
    echo "⚠️  Workspace ROS2 non trovato"
fi

if ros2 pkg list | grep -q ur_robot_driver; then
    echo "✅ ur_robot_driver disponibile"
    ros2 pkg prefix ur_robot_driver
else
    echo "❌ ur_robot_driver NON disponibile"
    echo "💡 Installa o compila il driver"
    exit 1
fi
echo

# PASSO 3: Setup Robot (istruzioni)
echo "================================================================================"
echo "PASSO 3: SETUP ROBOT (TEACH PENDANT) - SEGUI QUESTE ISTRUZIONI"
echo "================================================================================"
echo
echo "Secondo documentazione ufficiale GitHub:"
echo
echo "1. INSTALLA External Control URCap sul robot"
echo "   - Vai su: Installation → URCaps"
echo "   - Installa 'External Control'"
echo
echo "2. CREA PROGRAMMA con External Control"
echo "   - Crea nuovo programma"
echo "   - Aggiungi nodo 'External Control'"
echo "   - Configura IP Host: 192.168.10.191 (IP AI Accelerator)"
echo "   - Porta: 50002 (default)"
echo "   - Salva programma"
echo
echo "3. ESTRAI CALIBRAZIONE (IMPORTANTE per TCP pose corretto)"
echo "   Esegui questo comando DOPO aver configurato il robot:"
echo
echo "   ros2 launch ur_calibration calibration_correction.launch.py \\"
echo "       robot_ip:=${ROBOT_IP} \\"
echo "       target_filename:=\${HOME}/my_robot_calibration.yaml"
echo
echo "4. AVVIA PROGRAMMA sul teach pendant"
echo "   - Metti programma in PLAYING"
echo "   - Verifica che 'Remote Control' sia attivo"
echo
echo "================================================================================"
read -p "Premi INVIO quando hai completato il setup robot sul teach pendant..."
echo

# PASSO 4: Verifica stato robot
echo "PASSO 4: VERIFICA STATO ROBOT"
echo "------------------------------"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"robotmode\n")
    robot_mode = sock.recv(1024).decode().strip()
    print(f"   Robot Mode: {robot_mode}")
    
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    print(f"   Program State: {program_state}")
    
    sock.close()
    
    if "PLAYING" not in program_state:
        print()
        print("   ❌ Programma NON in PLAYING!")
        print("   💡 Avvia il programma sul teach pendant")
        exit(1)
    else:
        print("   ✅ Programma in PLAYING")
        
except Exception as e:
    print(f"   ❌ Errore: {e}")
    exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "Configura prima il robot con External Control URCap!"
    exit 1
fi
echo

# PASSO 5: Avvia driver (secondo documentazione ufficiale)
echo "================================================================================"
echo "PASSO 5: AVVIA DRIVER (SECONDO DOCUMENTAZIONE UFFICIALE)"
echo "================================================================================"
echo
echo "Comando ufficiale dalla documentazione GitHub:"
echo
echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "    ur_type:=${ROBOT_TYPE} \\"
echo "    robot_ip:=${ROBOT_IP}"
echo
echo "================================================================================"
echo "AVVIO DRIVER..."
echo "================================================================================"
echo

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=${ROBOT_TYPE} \
    robot_ip:=${ROBOT_IP} \
    launch_rviz:=false










