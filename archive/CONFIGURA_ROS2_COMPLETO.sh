#!/bin/bash
# Script completo per configurare ROS2 Driver secondo documentazione ufficiale

set -e

echo "================================================================================"
echo "CONFIGURAZIONE ROS2 DRIVER - DOCUMENTAZIONE UFFICIALE UNIVERSAL ROBOTS"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"
AI_ACCELERATOR_IP="192.168.10.191"

cd ~/MekoAiAccelerator || exit 1

# 1. Verifica ROS2
echo "1. Verifica ROS2..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "   ✅ ROS2 workspace configurato"
else
    echo "   ❌ Workspace ROS2 non trovato!"
    echo "   💡 Crea workspace: mkdir -p ~/ros2_ws/src"
    exit 1
fi
echo

# 2. Verifica driver UR installato
echo "2. Verifica driver UR ROS2..."
if ros2 pkg list | grep -q ur_robot_driver; then
    echo "   ✅ ur_robot_driver installato"
    DRIVER_PATH=$(ros2 pkg prefix ur_robot_driver)
    echo "   Path: $DRIVER_PATH"
else
    echo "   ❌ ur_robot_driver NON installato"
    echo "   💡 Installa con:"
    echo "      cd ~/ros2_ws/src"
    echo "      git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git"
    echo "      cd ~/ros2_ws"
    echo "      rosdep update"
    echo "      rosdep install --from-paths src --ignore-src -r -y"
    echo "      colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
    exit 1
fi
echo

# 3. Verifica rclpy
echo "3. Verifica rclpy..."
if python3 -c "import rclpy" 2>/dev/null; then
    echo "   ✅ rclpy disponibile"
else
    echo "   📦 Installo rclpy..."
    sudo apt install -y ros-humble-rclpy
    echo "   ✅ rclpy installato"
fi
echo

# 4. Verifica stato robot
echo "4. Verifica stato robot..."
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

# 5. Verifica porta 50002
echo "5. Verifica porta 50002..."
if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/50002" 2>/dev/null; then
    echo "   ✅ Porta 50002 APERTA - Robot pronto per ROS2!"
else
    echo "   ⚠️  Porta 50002 CHIUSA"
    echo "   💡 Questo è normale se:"
    echo "      - External Control URCap non è configurato sul robot"
    echo "      - Il programma con External Control non è in PLAYING"
    echo "      - Il driver UR ROS2 non è ancora in esecuzione"
fi
echo

# 6. Istruzioni per configurazione robot
echo "================================================================================"
echo "CONFIGURAZIONE ROBOT (TEACH PENDANT) - OBBLIGATORIA PER ROS2"
echo "================================================================================"
echo
echo "1. INSTALLA External Control URCap:"
echo "   - Vai su: Installation → URCaps"
echo "   - Installa 'External Control'"
echo
echo "2. CREA PROGRAMMA con External Control:"
echo "   - Crea nuovo programma"
echo "   - Aggiungi nodo 'External Control'"
echo "   - Configura:"
echo "     * IP Host: $AI_ACCELERATOR_IP"
echo "     * Porta: 50002"
echo "   - Salva come 'ros_control.urp'"
echo
echo "3. AVVIA PROGRAMMA:"
echo "   - Metti in PLAYING"
echo "   - Verifica che 'Remote Control' sia attivo"
echo
echo "================================================================================"
echo "AVVIO DRIVER UR ROS2"
echo "================================================================================"
echo
echo "Dopo aver configurato il robot, avvia il driver in un terminale separato:"
echo
echo "TERMINALE 1 - Driver UR ROS2:"
echo "-----------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "    ur_type:=${ROBOT_TYPE} \\"
echo "    robot_ip:=${ROBOT_IP} \\"
echo "    launch_rviz:=false"
echo
echo "Aspetta che vedi:"
echo "  [INFO] [ur_robot_driver]: Robot connected"
echo "  [INFO] [ur_robot_driver]: Controllers started"
echo
echo "================================================================================"
echo "VERIFICA CONTROLLER ROS2"
echo "================================================================================"
echo
echo "In un altro terminale, verifica che i controller siano attivi:"
echo
echo "TERMINALE 2 - Verifica Controller:"
echo "-----------------------------------"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "sudo apt install ros-humble-ros2controlcli  # se non installato"
echo "ros2 control list_controllers"
echo
echo "Dovresti vedere:"
echo "  scaled_joint_trajectory_controller [active]"
echo "  forward_velocity_controller [active]"
echo
echo "================================================================================"
echo "TEST MOVIMENTO ROS2"
echo "================================================================================"
echo
echo "Per testare che ROS2 funzioni:"
echo
echo "ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"
echo
echo "Il robot dovrebbe muoversi dopo pochi secondi."
echo
echo "================================================================================"
echo "MODIFICA WEB INTERFACE PER USARE ROS2"
echo "================================================================================"
echo
echo "Dopo che il driver è avviato e funzionante, modifica web_interface.py"
echo "per usare ROS2 invece di socket diretto."
echo
echo "================================================================================"










