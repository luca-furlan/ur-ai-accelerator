#!/bin/bash
# Passo 5: Riavvia web interface con ROS2 configurato

echo "================================================================================"
echo "PASSO 5: RIAVVIA WEB INTERFACE CON ROS2"
echo "================================================================================"
echo

cd ~/MekoAiAccelerator || exit 1

# Verifica che driver ROS2 sia in esecuzione
echo "1. Verifica driver ROS2..."
if pgrep -f "ur_robot_driver" > /dev/null; then
    echo "   ✅ Driver UR ROS2 in esecuzione"
else
    echo "   ⚠️  Driver UR ROS2 NON in esecuzione"
    echo "   💡 Avvia prima il driver (PASSO 3)"
    exit 1
fi
echo

# Ferma web interface esistente
echo "2. Fermo web interface esistente..."
pkill -f web_interface
sleep 2
echo "   ✅ Web interface fermata"
echo

# Source ROS2
echo "3. Configurazione ROS2..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "   ✅ ROS2 workspace configurato"
else
    echo "   ⚠️  Workspace ROS2 non trovato"
fi
echo

# Configura variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
export WEB_HOST=0.0.0.0

# Avvia web interface
echo "4. Avvio web interface con ROS2..."
echo "   URL: http://192.168.10.191:8081"
echo
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
PID=$!
echo $PID > /tmp/web_interface.pid

sleep 3

# Verifica
if ps -p $PID > /dev/null 2>&1; then
    echo "   ✅ Web interface avviata (PID: $PID)"
    echo "   Log: /tmp/web_interface.log"
    echo
    echo "   Ultime righe del log:"
    tail -15 /tmp/web_interface.log
    echo
    echo "================================================================================"
    echo "✅ WEB INTERFACE AVVIATA CON ROS2!"
    echo "================================================================================"
    echo
    echo "Ora:"
    echo "1. Apri browser: http://192.168.10.191:8081"
    echo "2. Muovi joystick"
    echo "3. Dovresti vedere 'ROS2 speedj' invece di 'Socket control'"
    echo "4. Il robot dovrebbe muoversi!"
    echo
else
    echo "   ❌ Web interface non avviata"
    echo "   Log completo:"
    cat /tmp/web_interface.log
    exit 1
fi










