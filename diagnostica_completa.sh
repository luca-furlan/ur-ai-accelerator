#!/bin/bash
# Diagnostica completa del sistema

echo "=========================================="
echo "DIAGNOSTICA COMPLETA SISTEMA"
echo "=========================================="
echo ""

# 1. Verifica robot
echo "1. VERIFICA ROBOT..."
export UR_ROBOT_IP=192.168.10.194
if ping -c 1 $UR_ROBOT_IP > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile"
    
    # Verifica porte
    if timeout 2 bash -c "</dev/tcp/$UR_ROBOT_IP/30002" 2>/dev/null; then
        echo "   ✅ Porta 30002 (URScript) raggiungibile"
    else
        echo "   ❌ Porta 30002 NON raggiungibile"
    fi
    
    if timeout 2 bash -c "</dev/tcp/$UR_ROBOT_IP/29999" 2>/dev/null; then
        echo "   ✅ Porta 29999 (Dashboard) raggiungibile"
        
        # Verifica stato robot
        echo ""
        echo "   Stato robot (Dashboard):"
        python3 << 'PYTHON'
import socket
import time
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    sock.connect(("192.168.10.194", 29999))
    sock.recv(1024)  # Welcome
    
    sock.sendall(b"robotmode\n")
    time.sleep(0.2)
    robotmode = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      Robot Mode: {robotmode}")
    
    sock.sendall(b"programState\n")
    time.sleep(0.2)
    program_state = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      Program State: {program_state}")
    
    sock.sendall(b"is in remote control\n")
    time.sleep(0.2)
    remote = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      Remote Control: {remote}")
    
    sock.close()
except Exception as e:
    print(f"      ❌ Errore: {e}")
PYTHON
    else
        echo "   ❌ Porta 29999 NON raggiungibile"
    fi
else
    echo "   ❌ Robot NON raggiungibile"
fi

# 2. Verifica ROS2
echo ""
echo "2. VERIFICA ROS2..."
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "   ❌ ROS2 non disponibile"; exit 1; }
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

if ros2 node list 2>/dev/null | grep -q ur; then
    echo "   ✅ Driver UR ROS2 in esecuzione"
    ros2 node list 2>/dev/null | grep ur
else
    echo "   ❌ Driver UR ROS2 NON in esecuzione"
    echo "   💡 Avvia: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194"
fi

# 3. Verifica topic
echo ""
echo "3. VERIFICA TOPIC ROS2..."
if ros2 topic list 2>/dev/null | grep -q forward_velocity_controller; then
    echo "   ✅ Topic /forward_velocity_controller/commands presente"
    
    # Verifica se riceve messaggi
    echo ""
    echo "   Test pubblicazione (5 secondi)..."
    timeout 5 ros2 topic echo /forward_velocity_controller/commands --once 2>/dev/null && echo "   ✅ Topic attivo e riceve messaggi" || echo "   ⚠️  Topic presente ma non riceve messaggi"
else
    echo "   ❌ Topic /forward_velocity_controller/commands NON presente"
    echo "   💡 Il driver UR non è configurato correttamente"
fi

# 4. Verifica web interface
echo ""
echo "4. VERIFICA WEB INTERFACE..."
if pgrep -f web_interface > /dev/null; then
    echo "   ✅ Web interface in esecuzione"
    ps aux | grep web_interface | grep -v grep | head -1
else
    echo "   ❌ Web interface NON in esecuzione"
fi

# 5. Test comando diretto
echo ""
echo "5. TEST COMANDO DIRETTO..."
echo "   Invio comando speedj di test..."
python3 << 'PYTHON'
import socket
import time
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    sock.connect(("192.168.10.194", 30002))
    
    # Comando speedj molto piccolo (sicuro)
    script = "speedj([0.01, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\n"
    sock.sendall(script.encode('utf-8'))
    time.sleep(0.5)
    sock.close()
    print("   ✅ Comando inviato (joint 0 +0.01 rad/s)")
    print("   💡 Controlla se il robot si muove leggermente")
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON

echo ""
echo "=========================================="
echo "RIEPILOGO"
echo "=========================================="
echo ""
echo "Se il robot non si muove, verifica:"
echo "1. ✅ Robot in modalità RUNNING (non POWER_OFF)"
echo "2. ✅ Programma sul teach pendant in PLAYING"
echo "3. ✅ Driver UR ROS2 in esecuzione"
echo "4. ✅ Topic /forward_velocity_controller/commands attivo"
echo "5. ✅ Web interface pubblica comandi (vedi log)"
echo ""

