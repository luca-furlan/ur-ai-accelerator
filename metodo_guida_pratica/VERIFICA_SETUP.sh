#!/bin/bash
# Script per verificare che tutto sia configurato correttamente

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROBOT_IP="192.168.10.194"

echo "================================================================================"
echo "VERIFICA SETUP COMPLETO"
echo "================================================================================"
echo

# Verifica 1: URCap scaricato
echo "1. VERIFICA URCAP SCARICATO"
echo "---------------------------"
if [ -f "externalcontrol-1.0.5.urcap" ]; then
    echo "✅ URCap presente: externalcontrol-1.0.5.urcap"
    echo "   Dimensione: $(du -h externalcontrol-1.0.5.urcap | cut -f1)"
else
    echo "❌ URCap NON presente"
    echo "   Esegui: ./SCARICA_URCAP.sh"
fi
echo

# Verifica 2: ROS2 installato
echo "2. VERIFICA ROS2 INSTALLATO"
echo "---------------------------"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble installato"
else
    echo "❌ ROS2 Humble NON installato"
fi
echo

# Verifica 3: Driver ROS2 disponibile
echo "3. VERIFICA DRIVER ROS2 DISPONIBILE"
echo "------------------------------------"
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

if ros2 pkg list 2>/dev/null | grep -q "ur_robot_driver"; then
    echo "✅ Driver ROS2 disponibile"
    PKG_PATH=$(ros2 pkg prefix ur_robot_driver 2>/dev/null || echo "N/A")
    echo "   Path: $PKG_PATH"
else
    echo "❌ Driver ROS2 NON disponibile"
    echo "   Installa con: sudo apt-get install ros-humble-ur"
fi
echo

# Verifica 4: Remote Control
echo "4. VERIFICA REMOTE CONTROL"
echo "--------------------------"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"is in remote control\n")
    remote_control = sock.recv(1024).decode().strip()
    
    if "true" in remote_control.lower():
        print("   ✅ Remote Control abilitato")
    else:
        print("   ❌ Remote Control NON abilitato")
        print("   💡 Abilita: Settings -> System -> Remote Control -> Enable")
    
    sock.close()
except Exception as e:
    print(f"   ⚠️  Errore connessione: {e}")
PYTHON
echo

# Verifica 5: Porta 50002
echo "5. VERIFICA PORTA 50002 (EXTERNAL CONTROL)"
echo "------------------------------------------"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ROBOT_IP, 50002))
    sock.close()
    
    if result == 0:
        print("   ✅ Porta 50002 APERTA")
        print("   ✅ External Control configurato correttamente")
    else:
        print("   ❌ Porta 50002 CHIUSA")
        print("   💡 Configura il programma sul Teach Pendant e mettilo in PLAYING")
except Exception as e:
    print(f"   ⚠️  Errore: {e}")
PYTHON
echo

# Verifica 6: Stato robot
echo "6. VERIFICA STATO ROBOT"
echo "-----------------------"
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
except Exception as e:
    print(f"   ⚠️  Errore: {e}")
PYTHON
echo

# Riepilogo
echo "================================================================================"
echo "RIEPILOGO"
echo "================================================================================"
echo
echo "Se tutte le verifiche sono OK, puoi procedere con:"
echo "  ./SETUP_COMPLETO.sh"
echo










