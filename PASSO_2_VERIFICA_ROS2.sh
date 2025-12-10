#!/bin/bash
# Passo 2: Verifica che tutto sia configurato correttamente per ROS2

echo "================================================================================"
echo "PASSO 2: VERIFICA CONFIGURAZIONE ROS2"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"

# 1. Verifica ROS2
echo "1. Verifica ROS2..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "   ✅ ROS2 workspace configurato"
else
    echo "   ❌ Workspace ROS2 non trovato!"
    exit 1
fi
echo

# 2. Verifica driver installato
echo "2. Verifica driver UR ROS2..."
if ros2 pkg list | grep -q ur_robot_driver; then
    echo "   ✅ ur_robot_driver installato"
else
    echo "   ❌ ur_robot_driver NON installato!"
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
        print("   ⚠️  ATTENZIONE: Programma NON in PLAYING!")
        print("   💡 Vai sul teach pendant e:")
        print("      1. Assicurati che External Control URCap sia installato")
        print("      2. Crea programma con nodo External Control")
        print("      3. IP Host: 192.168.10.191, Porta: 50002")
        print("      4. Avvia programma in PLAYING")
        exit(1)
    else:
        print("   ✅ Programma in PLAYING")
        
except Exception as e:
    print(f"   ❌ Errore: {e}")
    exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "================================================================================"
    echo "❌ CONFIGURA PRIMA IL ROBOT!"
    echo "================================================================================"
    exit 1
fi

echo

# 4. Verifica porta 50002
echo "4. Verifica porta 50002 (ROS2 Control)..."
if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/50002" 2>/dev/null; then
    echo "   ✅ Porta 50002 APERTA - Robot pronto per ROS2!"
else
    echo "   ⚠️  Porta 50002 CHIUSA"
    echo "   💡 Verifica che:"
    echo "      - External Control URCap sia configurato sul robot"
    echo "      - Programma con External Control sia in PLAYING"
    echo "      - IP Host: 192.168.10.191, Porta: 50002"
fi
echo

# 5. Verifica se driver è già in esecuzione
echo "5. Verifica driver ROS2 in esecuzione..."
if pgrep -f "ur_robot_driver" > /dev/null; then
    echo "   ✅ Driver UR ROS2 già in esecuzione"
    echo
    echo "   Verifica topic ROS2 disponibili..."
    ros2 topic list 2>&1 | head -20
    echo
    echo "   Verifica controller attivi..."
    if command -v ros2controlcli &> /dev/null || ros2 pkg list | grep -q ros2controlcli; then
        ros2 control list_controllers 2>&1 | head -10
    else
        echo "   ⚠️  ros2controlcli non installato"
        echo "   💡 Installa con: sudo apt install ros-humble-ros2controlcli"
    fi
else
    echo "   ⚠️  Driver UR ROS2 NON in esecuzione"
    echo
    echo "================================================================================"
    echo "PASSO 3: AVVIA DRIVER UR ROS2"
    echo "================================================================================"
    echo
    echo "Esegui questo comando:"
    echo
    echo "source /opt/ros/humble/setup.bash"
    echo "source ~/ros2_ws/install/setup.bash"
    echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
    echo "    ur_type:=ur5e \\"
    echo "    robot_ip:=192.168.10.194 \\"
    echo "    launch_rviz:=false"
    echo
    echo "Aspetta che vedi:"
    echo "  [INFO] [ur_robot_driver]: Robot connected"
    echo "  [INFO] [ur_robot_driver]: Controllers started"
    echo
fi

echo
echo "================================================================================"
echo "PROSSIMI PASSI"
echo "================================================================================"
echo
echo "Se il driver è già in esecuzione e vedi topic ROS2:"
echo "  → Vai al PASSO 4: Test controllo ROS2"
echo
echo "Se il driver NON è in esecuzione:"
echo "  → Vai al PASSO 3: Avvia driver UR ROS2"
echo




