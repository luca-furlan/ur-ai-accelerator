#!/bin/bash
# Script completo di diagnostica per robot che non si muove

echo "=================================================================================="
echo "🔍 DIAGNOSTICA COMPLETA: Robot Non Si Muove"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

echo "1. ✅ Verifica driver ROS2 attivo..."
if pgrep -f ur_ros2_control_node > /dev/null; then
    echo "   ✅ Driver ROS2 ATTIVO (PID: $(pgrep -f ur_ros2_control_node | head -1))"
else
    echo "   ❌ Driver ROS2 NON ATTIVO"
    echo "   💡 Avvia il driver ROS2 prima di continuare"
    exit 1
fi

echo ""
echo "2. ✅ Verifica porta 50002..."
if netstat -tuln 2>/dev/null | grep -q ":50002" || ss -tuln 2>/dev/null | grep -q ":50002"; then
    echo "   ✅ Porta 50002 in ascolto"
else
    echo "   ❌ Porta 50002 NON in ascolto"
    echo "   💡 Il driver ROS2 potrebbe non essere completamente avviato"
fi

echo ""
echo "3. ✅ Verifica controller attivo..."
CONTROLLER_STATUS=$(ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 2 "scaled_joint_trajectory_controller" | grep state | head -1)

if echo "$CONTROLLER_STATUS" | grep -q "active"; then
    echo "   ✅ scaled_joint_trajectory_controller ATTIVO"
elif echo "$CONTROLLER_STATUS" | grep -q "inactive"; then
    echo "   ❌ scaled_joint_trajectory_controller INATTIVO"
    echo "   💡 Attiva il controller con:"
    echo "      ros2 control switch_controllers --activate scaled_joint_trajectory_controller"
else
    echo "   ⚠️  Stato controller sconosciuto"
fi

echo ""
echo "4. ✅ Verifica topic trajectory..."
if ros2 topic list 2>/dev/null | grep -q "scaled_joint_trajectory_controller/joint_trajectory"; then
    echo "   ✅ Topic trajectory esiste"
    TOPIC_INFO=$(ros2 topic info /scaled_joint_trajectory_controller/joint_trajectory 2>&1)
    echo "$TOPIC_INFO" | grep -E "Publisher|Subscription" | head -2
else
    echo "   ❌ Topic trajectory NON esiste"
fi

echo ""
echo "5. ✅ Verifica stato robot..."
python3 << 'PYTHON'
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect(("192.168.10.194", 29999))
    sock.recv(1024)
    
    sock.sendall(b"robotmode\n")
    mode = sock.recv(1024).decode().strip()
    print(f"   Robot Mode: {mode}")
    
    sock.sendall(b"programState\n")
    state = sock.recv(1024).decode().strip()
    print(f"   Program State: {state}")
    
    sock.sendall(b"safetymode\n")
    safety = sock.recv(1024).decode().strip()
    print(f"   Safety Mode: {safety}")
    
    sock.close()
    
    if "RUNNING" in mode and ("PLAYING" in state or "RUNNING" in state):
        print("   ✅ Robot pronto")
    else:
        print("   ❌ Robot NON pronto")
        print("   💡 Assicurati che:")
        print("      - Robot Mode = RUNNING")
        print("      - Program State = PLAYING")
        print("      - Safety Mode = NORMAL")
except Exception as e:
    print(f"   ❌ Errore connessione robot: {e}")
PYTHON

echo ""
echo "6. ✅ Test pubblicazione messaggi..."
echo "   💡 MUOVI IL JOYSTICK nella web interface ora..."
echo "   💡 Monitoro per 5 secondi..."
echo ""

timeout 5 ros2 topic echo /scaled_joint_trajectory_controller/joint_trajectory 2>&1 | \
    grep -E "positions:|velocities:|time_from_start" | head -20 || \
    echo "   ⚠️  Nessun messaggio ricevuto (potrebbe essere normale se joystick fermo)"

echo ""
echo "7. ✅ Verifica bridge ROS2..."
if pgrep -f "ros2_bridge_fixed\|web_interface" > /dev/null; then
    echo "   ✅ Bridge ROS2/web interface ATTIVO"
    echo "   PID: $(pgrep -f 'ros2_bridge_fixed\|web_interface' | head -1)"
else
    echo "   ❌ Bridge ROS2/web interface NON ATTIVO"
fi

echo ""
echo "=================================================================================="
echo "📋 RISULTATO"
echo "=================================================================================="
echo ""
echo "Se tutti i controlli sono ✅:"
echo "  - Il problema potrebbe essere nelle velocità troppo basse"
echo "  - Verifica Speed Scaling sul Teach Pendant (deve essere > 0%)"
echo "  - Prova ad aumentare la velocità nel joystick"
echo ""
echo "Se qualche controllo è ❌:"
echo "  - Risolvi i problemi indicati sopra"
echo "  - Riavvia il driver ROS2 se necessario"
echo ""







