#!/bin/bash

# Script per avviare il driver ROS2 UR5e
# Questo DEVE essere avviato PRIMA di attivare Remote Control sul Teach Pendant

set -e

ROBOT_IP="${UR_ROBOT_IP:-192.168.10.194}"

echo "=================================================================================="
echo "🚀 AVVIO DRIVER ROS2 UR5e"
echo "=================================================================================="
echo ""
echo "Configurazione:"
echo "  Robot IP: $ROBOT_IP"
echo "  Porta 50002: Il driver aprirà questa porta per accettare connessioni dal robot"
echo ""

# Verifica se già in esecuzione
if pgrep -f "ur_ros2_control_node" > /dev/null; then
    echo "⚠️  Driver ROS2 già in esecuzione!"
    echo "   PID: $(pgrep -f 'ur_ros2_control_node' | head -1)"
    echo ""
    read -p "Vuoi fermarlo e riavviarlo? (s/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Ss]$ ]]; then
        echo "Fermo driver esistente..."
        pkill -f "ur_ros2_control_node" || true
        sleep 2
    else
        echo "Driver già attivo. Esco."
        exit 0
    fi
fi

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble configurato"
else
    echo "❌ ERRORE: ROS2 Humble non trovato in /opt/ros/humble/setup.bash"
    exit 1
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ Workspace ROS2 configurato"
else
    echo "⚠️  Workspace ROS2 non trovato in ~/ros2_ws/install/setup.bash"
fi

# Verifica porta 50002
if netstat -tuln 2>/dev/null | grep -q ":50002 "; then
    echo "⚠️  Porta 50002 già in uso!"
    echo "   Potrebbe essere un altro processo. Verifica con:"
    echo "   netstat -tuln | grep 50002"
    echo ""
    read -p "Vuoi continuare comunque? (s/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Ss]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "📋 ISTRUZIONI IMPORTANTI:"
echo ""
echo "1. Questo script avvierà il driver ROS2 che aprirà la porta 50002"
echo "2. DOPO che vedi 'System successfully started!' nel log:"
echo "   - Vai sul Teach Pendant"
echo "   - Attiva Remote Control"
echo "   - Avvia il programma External Control (porta 50002)"
echo "   - Il robot si connetterà automaticamente al driver ROS2"
echo ""
echo "3. In un altro terminale, avvia la web interface:"
echo "   cd ~/MekoAiAccelerator"
echo "   ./avvia_web_interface_joystick.sh"
echo ""
echo "⚠️  SICUREZZA:"
echo "   - Assicurati che l'area di lavoro sia libera"
echo "   - Tieni pronto l'e-stop fisico del robot"
echo ""
echo "Premi CTRL+C per fermare il driver"
echo ""
echo "=================================================================================="
echo "🚀 Avvio driver ROS2..."
echo "=================================================================================="
echo ""

# Avvia driver ROS2
cd ~/MekoAiAccelerator
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false

