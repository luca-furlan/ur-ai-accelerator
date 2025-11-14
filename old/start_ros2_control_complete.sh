#!/bin/bash
# Script completo per avviare ROS2 UR Driver + Web Interface

set -e

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"
AI_ACCELERATOR_IP="192.168.10.191"

echo "=========================================="
echo "AVVIO ROS2 UR CONTROL COMPLETO"
echo "=========================================="
echo ""

cd ~/MekoAiAccelerator

# 1. Installa rclpy se necessario
echo "1. Verifica rclpy..."
if ! python3 -c "import rclpy" 2>/dev/null; then
    echo "   Installazione rclpy..."
    pip3 install --user rclpy || {
        echo "   ⚠️ Fallito installazione rclpy - continua comunque"
    }
else
    echo "   ✅ rclpy già installato"
fi

# 2. Source ROS2
echo ""
echo "2. Setup ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "   ✅ ROS2 Humble sourced"
    
    # Source workspace se esiste
    if [ -d ~/ur_ws/install ]; then
        source ~/ur_ws/install/setup.bash
        echo "   ✅ UR workspace sourced"
    elif [ -d ~/ros2_ws/install ]; then
        source ~/ros2_ws/install/setup.bash
        echo "   ✅ ROS2 workspace sourced"
    fi
else
    echo "   ❌ ROS2 non trovato!"
    exit 1
fi

# 3. Verifica driver UR
echo ""
echo "3. Verifica UR Driver..."
if ros2 pkg list | grep ur_robot_driver > /dev/null 2>&1; then
    echo "   ✅ UR Robot Driver disponibile"
else
    echo "   ⚠️ UR Robot Driver non trovato - potrebbe non funzionare"
fi

# 4. Verifica connessione robot
echo ""
echo "4. Verifica connessione robot..."
if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/30002" 2>/dev/null; then
    echo "   ✅ Robot raggiungibile"
else
    echo "   ⚠️ Robot NON raggiungibile - verifica connessione"
fi

# 5. Termina processi esistenti
echo ""
echo "5. Pulizia processi esistenti..."
pkill -f "ur_control.launch.py" 2>/dev/null || true
pkill -f "web_interface" 2>/dev/null || true
sleep 1
echo "   ✅ Pulizia completata"

# 6. Avvia UR Driver ROS2 in background
echo ""
echo "6. Avvio UR Driver ROS2..."
export UR_ROBOT_IP=$ROBOT_IP
nohup ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$ROBOT_TYPE \
    robot_ip:=$ROBOT_IP \
    > ur_driver.log 2>&1 &
UR_DRIVER_PID=$!
echo "   ✅ UR Driver avviato (PID: $UR_DRIVER_PID)"
echo "   📋 Log: tail -f ur_driver.log"

# Attendi che ROS2 si inizializzi
echo "   ⏳ Attesa inizializzazione ROS2 (5 secondi)..."
sleep 5

# 7. Verifica topic ROS2
echo ""
echo "7. Verifica topic ROS2..."
if ros2 topic list | grep -q "servo_node"; then
    echo "   ✅ Servo node disponibile"
    ros2 topic list | grep servo_node | head -3
else
    echo "   ⚠️ Servo node non trovato - potrebbe non essere ancora inizializzato"
fi

# 8. Avvia Web Interface
echo ""
echo "=========================================="
echo "AVVIO WEB INTERFACE"
echo "=========================================="
echo ""
echo "🌐 Web Interface: http://$AI_ACCELERATOR_IP:8080"
echo "📋 Log UR Driver: tail -f ur_driver.log"
echo "📋 Log Web Interface: tail -f web_interface.log"
echo ""
echo "Premi CTRL+C per fermare tutto"
echo ""

export UR_ROBOT_IP=$ROBOT_IP
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Funzione cleanup
cleanup() {
    echo ""
    echo "=========================================="
    echo "FERMATURA IN CORSO..."
    echo "=========================================="
    kill $UR_DRIVER_PID 2>/dev/null || true
    pkill -f "ur_control.launch.py" 2>/dev/null || true
    pkill -f "web_interface" 2>/dev/null || true
    echo "✅ Tutto fermato"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Avvia web interface
python3 -m remote_ur_control.web_interface 2>&1 | tee web_interface.log

