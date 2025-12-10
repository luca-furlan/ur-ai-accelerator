#!/bin/bash
# Debug crash driver ROS2 - cerca problemi noti

set -e

echo "================================================================================"
echo "DEBUG CRASH DRIVER ROS2"
echo "================================================================================"
echo

# Verifica se driver è stato avviato
echo "1. VERIFICA PROCESSI ROS2"
echo "-------------------------"
ps aux | grep -E "(ur_robot_driver|ur_ros2_control|ros2 launch)" | grep -v grep || echo "   Nessun processo trovato"
echo

# Verifica log recenti
echo "2. LOG RECENTI ROS2"
echo "-------------------"
if [ -f ~/.ros/log/latest/ros2*.log ]; then
    echo "Ultimi errori nei log ROS2:"
    tail -100 ~/.ros/log/latest/ros2*.log 2>/dev/null | grep -iE "(error|fault|crash|segmentation|exception)" | tail -20 || echo "   Nessun errore trovato nei log"
else
    echo "   Log ROS2 non trovati"
fi
echo

# Verifica core dump
echo "3. VERIFICA CORE DUMP"
echo "---------------------"
if ls core* 2>/dev/null | head -1; then
    echo "⚠️  Core dump trovato!"
    ls -lh core* | head -5
else
    echo "   Nessun core dump trovato"
fi
echo

# Verifica dmesg per segmentation fault
echo "4. VERIFICA SEGMENTATION FAULT (dmesg)"
echo "--------------------------------------"
dmesg | tail -50 | grep -iE "(segfault|segmentation|ur_robot|ur_ros2)" || echo "   Nessun segfault recente trovato"
echo

# Verifica configurazione robot
echo "5. VERIFICA STATO ROBOT"
echo "------------------------"
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
    
    sock.sendall(b"safetymode\n")
    safety_mode = sock.recv(1024).decode().strip()
    print(f"   Safety Mode: {safety_mode}")
    
    sock.sendall(b"get loaded program\n")
    loaded_program = sock.recv(1024).decode().strip()
    print(f"   Loaded Program: {loaded_program}")
    
    sock.close()
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON

echo

# Verifica porta 50002 (External Control)
echo "6. VERIFICA PORTA EXTERNAL CONTROL (50002)"
echo "-------------------------------------------"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ROBOT_IP, 50002))
    sock.close()
    if result == 0:
        print("   ✅ Porta 50002 aperta (External Control attivo)")
    else:
        print("   ❌ Porta 50002 chiusa (External Control NON attivo)")
        print("   💡 Il programma External Control deve essere in PLAYING")
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON

echo

# Verifica RTDE
echo "7. VERIFICA RTDE (porta 30004)"
echo "-------------------------------"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ROBOT_IP, 30004))
    sock.close()
    if result == 0:
        print("   ✅ Porta 30004 aperta (RTDE disponibile)")
    else:
        print("   ⚠️  Porta 30004 chiusa")
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON

echo

# Verifica conflitti RTDE
echo "8. VERIFICA CONFLITTI RTDE"
echo "---------------------------"
RTDE_PROCESSES=$(ps aux | grep -E "(rtde|RTDE)" | grep -v grep | wc -l)
if [ $RTDE_PROCESSES -gt 0 ]; then
    echo "   ⚠️  Processi RTDE trovati:"
    ps aux | grep -E "(rtde|RTDE)" | grep -v grep
    echo "   💡 RTDE supporta solo UN client alla volta!"
else
    echo "   ✅ Nessun processo RTDE trovato"
fi
echo

# Verifica versione driver
echo "9. VERIFICA VERSIONE DRIVER"
echo "---------------------------"
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

if ros2 pkg prefix ur_robot_driver >/dev/null 2>&1; then
    PKG_PATH=$(ros2 pkg prefix ur_robot_driver)
    if [ -f "$PKG_PATH/share/ur_robot_driver/package.xml" ]; then
        VERSION=$(grep -oP '(?<=<version>)[^<]+' "$PKG_PATH/share/ur_robot_driver/package.xml" 2>/dev/null || echo "N/A")
        echo "   Versione driver: $VERSION"
    fi
fi
echo

echo "================================================================================"
echo "PROBLEMI NOTI DAL REPOSITORY GITHUB"
echo "================================================================================"
echo
echo "Cerca problemi comuni su:"
echo "https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues"
echo
echo "Problemi comuni:"
echo "1. C218A1 - External Control non configurato correttamente"
echo "2. Segmentation fault - Conflitto RTDE o versione driver"
echo "3. Porta 50002 chiusa - Programma non in PLAYING"
echo "4. Calibrazione mancante - TCP pose errato"
echo



