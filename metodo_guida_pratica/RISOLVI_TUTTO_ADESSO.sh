#!/bin/bash

# Script completo per risolvere tutti i problemi
# Basato su Issue #31 e #37

set +e  # Continua anche con errori

PC_IP="192.168.10.191"
ROBOT_IP="192.168.10.194"
PORT="50002"

echo "=================================================================================="
echo "🔧 RISOLUZIONE COMPLETA PROBLEMI EXTERNAL CONTROL"
echo "=================================================================================="
echo ""
echo "Basato su Issue #31 e #37 di GitHub"
echo ""

# 1. Ferma tutto
echo "1. Fermo tutti i processi ROS2 e RTDE..."
pkill -f "ur_robot_driver\|ur_control.launch\|ros2\|rtde\|ur_rtde" 2>/dev/null
sleep 3
echo "   ✅ Processi fermati"
echo ""

# 2. Apri porta nel firewall
echo "2. Apertura porta $PORT nel firewall..."
if command -v ufw > /dev/null 2>&1; then
    if sudo ufw status 2>/dev/null | grep -q "$PORT"; then
        echo "   ✅ Porta $PORT già aperta"
    else
        echo "   Aprendo porta $PORT..."
        if sudo ufw allow $PORT 2>&1; then
            sudo ufw reload 2>&1
            echo "   ✅ Porta $PORT aperta"
        else
            echo "   ⚠️  Impossibile aprire porta (serve sudo senza password)"
            echo "   Esegui manualmente: sudo ufw allow $PORT && sudo ufw reload"
        fi
    fi
else
    echo "   ⚠️  UFW non installato"
fi
echo ""

# 3. Verifica porta libera
echo "3. Verifica porta $PORT libera..."
if netstat -tuln | grep -q ":$PORT "; then
    echo "   ⚠️  Porta $PORT già in uso!"
    if command -v lsof > /dev/null 2>&1; then
        echo "   Processi che usano porta $PORT:"
        sudo lsof -i :$PORT 2>/dev/null || echo "   Impossibile verificare"
    fi
else
    echo "   ✅ Porta $PORT libera"
fi
echo ""

# 4. Avvia driver ROS2 in background
echo "4. Avvio driver ROS2..."
cd ~/MekoAiAccelerator/metodo_guida_pratica

source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo ""
echo "   Avvio driver ROS2 in background..."
echo "   Log: /tmp/driver_ros2_fix.log"
echo ""

# Avvia driver in background e cattura output
nohup ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false \
    > /tmp/driver_ros2_fix.log 2>&1 &

DRIVER_PID=$!
echo "   Driver avviato (PID: $DRIVER_PID)"
echo ""

# 5. Attendi e verifica
echo "5. Attendo avvio driver (10 secondi)..."
sleep 10

echo ""
echo "   Verifica driver attivo..."
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "   ✅ Driver ROS2 attivo"
else
    echo "   ❌ Driver ROS2 NON attivo (crash?)"
    echo ""
    echo "   Ultimi messaggi dal log:"
    tail -20 /tmp/driver_ros2_fix.log | grep -E "ERROR|FATAL|Segmentation|crash" || tail -10 /tmp/driver_ros2_fix.log
fi
echo ""

# 6. Verifica porta in ascolto
echo "6. Verifica porta $PORT in ascolto..."
sleep 2
if netstat -tuln | grep -q ":$PORT "; then
    echo "   ✅ PC IN ASCOLTO sulla porta $PORT!"
    netstat -tuln | grep ":$PORT "
else
    echo "   ❌ PC ancora NON in ascolto sulla porta $PORT"
    echo ""
    echo "   Possibili cause:"
    echo "   1. Driver ROS2 è crashato (vedi log sopra)"
    echo "   2. EtherNet/IP abilitato sul robot (Issue #31)"
    echo "   3. Driver non si è ancora avviato completamente"
fi
echo ""

# 7. Test dal robot
echo "7. Test connettività Robot → PC..."
echo "   (Simula quello che fa il robot)"
if timeout 3 nc -zv "$PC_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
    echo "   ✅ Robot può connettersi al PC sulla porta $PORT!"
else
    echo "   ❌ Robot NON può connettersi (Connection refused)"
    echo ""
    echo "   Questo è il problema che vedi sul Teach Pendant!"
fi
echo ""

# 8. Riepilogo
echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""

if netstat -tuln | grep -q ":$PORT "; then
    echo "✅ PC IN ASCOLTO sulla porta $PORT"
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
    echo "Il robot dovrebbe connettersi al PC!"
else
    echo "❌ PC NON in ascolto sulla porta $PORT"
    echo ""
    echo "PROBLEMA: Driver ROS2 va in crash"
    echo ""
    echo "SOLUZIONE:"
    echo "1. Verifica EtherNet/IP DISABILITATO sul robot (Issue #31)"
    echo "2. Verifica log driver: tail -f /tmp/driver_ros2_fix.log"
    echo "3. Se vedi 'Variable speed_slider_mask is currently controlled' → EtherNet/IP è abilitato!"
    echo ""
fi

echo ""
echo "=================================================================================="
echo ""
echo "Per monitorare driver:"
echo "  tail -f /tmp/driver_ros2_fix.log"
echo ""
echo "Per verificare porta:"
echo "  netstat -tuln | grep 50002"
echo ""








