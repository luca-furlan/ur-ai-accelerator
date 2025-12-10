#!/bin/bash

# Fix rapido per "Connection refused" sulla porta 50002
# Basato su Issue #31 e #37

PC_IP="192.168.10.191"
PORT="50002"

echo "=================================================================================="
echo "🔧 FIX RAPIDO: Connection Refused sulla porta $PORT"
echo "=================================================================================="
echo ""

# 1. Verifica macchina in ascolto
echo "1. Verifica macchina in ascolto..."
if netstat -tuln | grep -q ":$PORT "; then
    echo "   ✅ Macchina in ascolto sulla $PORT"
else
    echo "   ❌ Macchina NON in ascolto"
    echo ""
    echo "   Avvio driver ROS2..."
    cd ~/MekoAiAccelerator/metodo_guida_pratica
    if [ -f START_RAPIDO.sh ]; then
        echo "   Esegui: ./START_RAPIDO.sh"
    else
        echo "   Esegui: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false"
    fi
fi
echo ""

# 2. Apri porta nel firewall
echo "2. Apertura porta nel firewall..."
if command -v ufw > /dev/null 2>&1; then
    if sudo ufw status | grep -q "$PORT"; then
        echo "   ✅ Porta $PORT già aperta"
    else
        echo "   Aprendo porta $PORT..."
        sudo ufw allow $PORT
        sudo ufw reload
        echo "   ✅ Porta $PORT aperta"
    fi
else
    echo "   ⚠️  UFW non installato, verifica iptables manualmente"
fi
echo ""

# 3. Verifica driver ROS2
echo "3. Verifica driver ROS2..."
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "   ✅ Driver ROS2 attivo"
else
    echo "   ❌ Driver ROS2 NON attivo"
    echo "   Avvia con: ./START_RAPIDO.sh"
fi
echo ""

# 4. Test porta
echo "4. Test porta $PORT..."
if timeout 3 nc -zv "$PC_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
    echo "   ✅ Porta $PORT accessibile"
else
    echo "   ❌ Porta $PORT NON accessibile"
fi
echo ""

echo "=================================================================================="
echo "📋 VERIFICA SUL TEACH PENDANT"
echo "=================================================================================="
echo ""
echo "1. EtherNet/IP DISABILITATO (Installation → Fieldbus)"
echo "2. Remote Control abilitato (Settings → System → Remote Control)"
echo "3. External Control: Host IP = $PC_IP, Port = $PORT"
echo "4. Avvia programma DOPO che driver ROS2 è attivo"
echo ""
echo "=================================================================================="

