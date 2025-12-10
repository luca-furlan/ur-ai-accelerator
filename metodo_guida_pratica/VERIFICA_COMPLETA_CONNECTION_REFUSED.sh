#!/bin/bash

# Script per verificare tutte le cause di "Connection Refused"

echo "=================================================================================="
echo "🔍 VERIFICA COMPLETA: CONNECTION REFUSED"
echo "=================================================================================="
echo ""

ROBOT_IP="192.168.10.194"
PC_IP="192.168.10.191"
PORT="50002"

echo "1. VERIFICA CONNETTIVITÀ DI RETE"
echo "--------------------------------"
if ping -c 2 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile: $ROBOT_IP"
else
    echo "❌ Robot NON raggiungibile: $ROBOT_IP"
fi
echo ""

echo "2. VERIFICA MACCHINA REMOTA IN ASCOLTO"
echo "--------------------------------------"
if netstat -tuln | grep -q ":$PORT "; then
    echo "✅ Macchina remota in ascolto sulla porta $PORT"
    netstat -tuln | grep ":$PORT "
else
    echo "❌ Macchina remota NON in ascolto sulla porta $PORT"
    echo "   SOLUZIONE: Avvia driver ROS2 PRIMA!"
fi
echo ""

echo "3. VERIFICA FIREWALL"
echo "--------------------"
if command -v ufw > /dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | head -1)
    if echo "$UFW_STATUS" | grep -q "inactive\|Status: inactive"; then
        echo "✅ Firewall non attivo"
    else
        echo "⚠️  Firewall attivo: $UFW_STATUS"
        if sudo ufw status | grep -q "$PORT"; then
            echo "✅ Porta $PORT già aperta nel firewall"
        else
            echo "❌ Porta $PORT NON aperta nel firewall"
            echo "   SOLUZIONE: sudo ufw allow $PORT"
        fi
    fi
else
    echo "⚠️  UFW non installato, verifica firewall manualmente"
fi
echo ""

echo "4. VERIFICA PROCESSI CHE USANO PORTA 50002"
echo "------------------------------------------"
if command -v lsof > /dev/null 2>&1; then
    PROCESSES=$(sudo lsof -i :$PORT 2>/dev/null)
    if [ -z "$PROCESSES" ]; then
        echo "✅ Nessun processo usa porta $PORT"
    else
        echo "⚠️  Processi che usano porta $PORT:"
        echo "$PROCESSES"
    fi
else
    echo "⚠️  lsof non installato, verifica manualmente: sudo netstat -tulpn | grep $PORT"
fi
echo ""

echo "5. VERIFICA DRIVER ROS2 IN ESECUZIONE"
echo "--------------------------------------"
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "✅ Driver ROS2 in esecuzione"
    ps aux | grep "ur_robot_driver\|ur_control.launch" | grep -v grep | head -2
else
    echo "❌ Driver ROS2 NON in esecuzione"
    echo "   SOLUZIONE: Avvia driver ROS2 PRIMA!"
fi
echo ""

echo "6. VERIFICA CONNESSIONE DAL ROBOT"
echo "----------------------------------"
if timeout 3 nc -zv $ROBOT_IP $PORT 2>&1 | grep -q "succeeded\|open"; then
    echo "✅ Robot ha porta $PORT aperta"
else
    echo "❌ Robot NON ha porta $PORT aperta"
    echo "   SOLUZIONE: Avvia programma sul robot con External Control!"
fi
echo ""

echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "Per risolvere 'Connection Refused':"
echo ""
echo "1. Avvia driver ROS2 PRIMA:"
echo "   cd ~/MekoAiAccelerator/metodo_guida_pratica"
echo "   ./START_RAPIDO.sh"
echo ""
echo "2. Verifica macchina in ascolto:"
echo "   netstat -tuln | grep 50002"
echo ""
echo "3. Apri porta nel firewall (se necessario):"
echo "   sudo ufw allow 50002"
echo ""
echo "4. Configura Teach Pendant:"
echo "   - Host IP: $PC_IP"
echo "   - Porta: $PORT"
echo ""
echo "5. Avvia programma sul robot (PLAY)"
echo ""
echo "=================================================================================="

