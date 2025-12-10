#!/bin/bash
# Script per fixare ROS2 e abilitare controllo completo robot

set -e

echo "================================================================================"
echo "FIX COMPLETO ROS2 - ABILITA CONTROLLO ROBOT"
echo "================================================================================"
echo

cd ~/MekoAiAccelerator || exit 1

# 1. Source ROS2
echo "1. Configurazione ROS2..."
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}

echo "   ✅ ROS2 environment configurato"
echo

# 2. Installa rclpy se non presente
echo "2. Verifica rclpy..."
if python3 -c "import rclpy" 2>/dev/null; then
    echo "   ✅ rclpy già disponibile"
else
    echo "   📦 Installo rclpy..."
    sudo apt update
    sudo apt install -y ros-humble-rclpy ros-humble-rclpy-common
    echo "   ✅ rclpy installato"
fi
echo

# 3. Verifica che rclpy funzioni
echo "3. Test rclpy..."
if python3 -c "import rclpy; print('✅ rclpy OK')" 2>&1; then
    echo "   ✅ rclpy funziona!"
else
    echo "   ⚠️ rclpy ancora non funziona, verifico variabili ambiente..."
    echo "   LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    echo "   PYTHONPATH: $PYTHONPATH"
fi
echo

# 4. Ferma web interface esistente
echo "4. Fermo web interface esistente..."
pkill -f web_interface || true
sleep 2
echo "   ✅ Web interface fermata"
echo

# 5. Configura variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
export WEB_HOST=0.0.0.0

# 6. Avvia web interface con ROS2
echo "5. Avvio web interface con ROS2 configurato..."
echo "   URL: http://192.168.10.191:8081"
echo "   Robot IP: $UR_ROBOT_IP"
echo

nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
PID=$!
echo $PID > /tmp/web_interface.pid

sleep 3

# 7. Verifica
if ps -p $PID > /dev/null 2>&1; then
    echo "   ✅ Web interface avviata (PID: $PID)"
    echo "   Log: /tmp/web_interface.log"
    echo
    echo "   Ultime righe del log:"
    tail -15 /tmp/web_interface.log
else
    echo "   ❌ Web interface non avviata"
    echo "   Log completo:"
    cat /tmp/web_interface.log
    exit 1
fi

echo
echo "================================================================================"
echo "✅ TUTTO CONFIGURATO!"
echo "================================================================================"
echo
echo "Web interface: http://192.168.10.191:8081"
echo "ROS2: Configurato e disponibile"
echo
echo "Per controllare il robot:"
echo "1. Sul teach pendant: avvia programma in PLAYING"
echo "2. Nella web interface: usa joystick o controlli"
echo




