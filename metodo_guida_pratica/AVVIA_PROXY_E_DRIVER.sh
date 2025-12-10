#!/bin/bash

# Script per avviare proxy server e driver ROS2 insieme
# Il proxy gestisce le connessioni mentre il driver si avvia

set +e

PC_IP="192.168.10.191"
ROBOT_IP="192.168.10.194"
PORT="50002"

echo "=================================================================================="
echo "🚀 AVVIO PROXY SERVER + DRIVER ROS2"
echo "=================================================================================="
echo ""
echo "Questo script:"
echo "1. Avvia un proxy server sulla porta 50002"
echo "2. Avvia il driver ROS2"
echo "3. Il proxy gestisce le connessioni dal robot anche se il driver va in crash"
echo ""

cd ~/MekoAiAccelerator/metodo_guida_pratica

# Ferma tutto
echo "1. Fermo processi esistenti..."
pkill -f "ur_robot_driver\|ur_control.launch\|external_control_proxy" 2>/dev/null
sleep 2
echo "   ✅ Processi fermati"
echo ""

# Apri porta firewall
echo "2. Apertura porta $PORT nel firewall..."
if command -v ufw > /dev/null 2>&1; then
    if sudo ufw status 2>/dev/null | grep -q "$PORT"; then
        echo "   ✅ Porta già aperta"
    else
        if sudo ufw allow $PORT 2>&1; then
            sudo ufw reload 2>&1
            echo "   ✅ Porta aperta"
        else
            echo "   ⚠️  Impossibile aprire porta (serve sudo)"
            echo "   Esegui manualmente: sudo ufw allow $PORT && sudo ufw reload"
        fi
    fi
fi
echo ""

# Avvia proxy server in background
echo "3. Avvio proxy server..."
if [ -f external_control_proxy_server.py ]; then
    chmod +x external_control_proxy_server.py
    nohup python3 external_control_proxy_server.py > /tmp/proxy_server.log 2>&1 &
    PROXY_PID=$!
    echo "   ✅ Proxy server avviato (PID: $PROXY_PID)"
    echo "   Log: tail -f /tmp/proxy_server.log"
    sleep 2
else
    echo "   ❌ File external_control_proxy_server.py non trovato"
    exit 1
fi
echo ""

# Verifica proxy in ascolto
echo "4. Verifica proxy server..."
sleep 2
if netstat -tuln | grep -q ":$PORT "; then
    echo "   ✅ Proxy server IN ASCOLTO sulla porta $PORT!"
    netstat -tuln | grep ":$PORT "
else
    echo "   ⚠️  Proxy server non in ascolto"
    echo "   Verifica log: tail -f /tmp/proxy_server.log"
fi
echo ""

# Avvia driver ROS2
echo "5. Avvio driver ROS2..."
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo ""
echo "   Avvio driver ROS2..."
echo "   Log: /tmp/driver_ros2.log"
echo ""

# Avvia driver in background
nohup ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false \
    > /tmp/driver_ros2.log 2>&1 &

DRIVER_PID=$!
echo "   Driver avviato (PID: $DRIVER_PID)"
echo ""

# Attendi e verifica
echo "6. Attendo avvio driver (15 secondi)..."
sleep 15

echo ""
echo "   Verifica driver..."
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "   ✅ Driver ROS2 attivo"
else
    echo "   ⚠️  Driver ROS2 potrebbe essere crashato"
    echo "   Verifica log: tail -50 /tmp/driver_ros2.log"
fi
echo ""

# Riepilogo
echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "Proxy server:"
echo "  PID: $PROXY_PID"
echo "  Log: tail -f /tmp/proxy_server.log"
echo ""
echo "Driver ROS2:"
echo "  PID: $DRIVER_PID"
echo "  Log: tail -f /tmp/driver_ros2.log"
echo ""
echo "Verifica porte:"
echo "  netstat -tuln | grep 50002"
echo ""
echo "PROSSIMI PASSI SUL TEACH PENDANT:"
echo ""
echo "1. Verifica EtherNet/IP DISABILITATO (Installation → Fieldbus)"
echo "2. Verifica Remote Control abilitato (Settings → System → Remote Control)"
echo "3. Configura External Control:"
echo "   - Host IP: $PC_IP"
echo "   - Port: $PORT"
echo "4. Avvia programma (PLAY)"
echo ""
echo "Il proxy server gestirà la connessione anche se il driver va in crash!"
echo ""
echo "=================================================================================="
echo ""
echo "Per fermare tutto:"
echo "  pkill -f 'ur_robot_driver\|external_control_proxy'"
echo ""

