#!/bin/bash
# Script per configurare correttamente UR ROS2 Driver secondo documentazione ufficiale

set -e

echo "================================================================================"
echo "CONFIGURAZIONE UR ROS2 DRIVER - DOCUMENTAZIONE UFFICIALE"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

# 1. Verifica ROS2
echo "1. Verifica ROS2..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "   ✅ ROS2 workspace configurato"
else
    echo "   ⚠️  Workspace ROS2 non trovato"
fi
echo

# 2. Verifica driver UR installato
echo "2. Verifica driver UR ROS2..."
if ros2 pkg list | grep -q ur_robot_driver; then
    echo "   ✅ ur_robot_driver installato"
    ros2 pkg prefix ur_robot_driver
else
    echo "   ❌ ur_robot_driver NON installato"
    echo "   💡 Installa con: cd ~/ros2_ws/src && git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git"
    exit 1
fi
echo

# 3. Verifica stato robot
echo "3. Verifica stato robot..."
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
        print("   ⚠️  ATTENZIONE: Programma NON in PLAYING")
        print("   💡 Per usare ROS2 Driver:")
        print("      1. Sul teach pendant: Installa External Control URCap")
        print("      2. Crea programma con nodo External Control")
        print("      3. IP Host: 192.168.10.191, Porta: 50002")
        print("      4. Avvia programma in PLAYING")
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON
echo

# 4. Verifica controller ROS2
echo "4. Verifica controller ROS2..."
echo "   (Se il driver è in esecuzione, vedrai i controller attivi)"
if command -v ros2controlcli &> /dev/null || ros2 pkg list | grep -q ros2controlcli; then
    source /opt/ros/humble/setup.bash
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
    fi
    ros2 control list_controllers 2>&1 | head -10 || echo "   ⚠️  Nessun controller trovato (driver non in esecuzione?)"
else
    echo "   ⚠️  ros2controlcli non installato"
    echo "   💡 Installa con: sudo apt install ros-humble-ros2controlcli"
fi
echo

# 5. Istruzioni per avviare driver
echo "================================================================================"
echo "ISTRUZIONI PER AVVIARE DRIVER UR ROS2"
echo "================================================================================"
echo
echo "TERMINALE 1 - Avvia Driver UR ROS2:"
echo "-----------------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "    ur_type:=${ROBOT_TYPE} \\"
echo "    robot_ip:=${ROBOT_IP} \\"
echo "    launch_rviz:=false"
echo
echo "TERMINALE 2 - Test Movimento (dopo che driver è avviato):"
echo "----------------------------------------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"
echo
echo "TERMINALE 3 - Web Interface (con ROS2 attivo):"
echo "-----------------------------------------------"
echo "cd ~/MekoAiAccelerator"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "export UR_ROBOT_IP=${ROBOT_IP}"
echo "export WEB_PORT=8081"
echo "python3 -m remote_ur_control.web_interface"
echo
echo "================================================================================"
echo "CONFIGURAZIONE ROBOT (TEACH PENDANT)"
echo "================================================================================"
echo
echo "1. Installa External Control URCap sul robot"
echo "2. Crea programma con nodo External Control"
echo "3. Configura:"
echo "   - IP Host: 192.168.10.191 (IP AI Accelerator)"
echo "   - Porta: 50002"
echo "4. Salva programma (es. 'ros_control.urp')"
echo "5. Avvia programma in PLAYING"
echo
echo "================================================================================"




