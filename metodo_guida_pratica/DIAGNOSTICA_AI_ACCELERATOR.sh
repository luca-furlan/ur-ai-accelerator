#!/bin/bash

# Script di diagnostica specifica per AI Accelerator
# Esegui questo script SULL'AI ACCELERATOR

PC_ACCELERATOR_IP="192.168.10.191"
PC_ACCELERATOR_USER="lab"
ROBOT_IP="192.168.10.194"

echo "=================================================================================="
echo "🔍 DIAGNOSTICA AI ACCELERATOR ($PC_ACCELERATOR_IP)"
echo "=================================================================================="
echo ""

# Informazioni sistema
echo "1. INFORMAZIONI SISTEMA"
echo "----------------------"
echo "Hostname: $(hostname)"
echo "IP: $(hostname -I)"
echo "OS: $(uname -a)"
echo ""

# Interfacce di rete
echo "2. INTERFACCE DI RETE"
echo "---------------------"
ip addr show | grep -E "^[0-9]+:|inet " | head -10
echo ""

# Routing
echo "3. ROUTING"
echo "----------"
ip route | head -5
echo ""

# Porte in ascolto
echo "4. PORTE IN ASCOLTO"
echo "-------------------"
echo "Porte rilevanti (50001-50004, 22):"
netstat -tuln | grep -E "5000[1-4]|22" || echo "Nessuna porta rilevante in ascolto"
echo ""

# Processi ROS2
echo "5. PROCESSI ROS2"
echo "----------------"
if pgrep -f "ur_robot_driver\|ros2\|ur_control" > /dev/null; then
    echo "✅ Processi ROS2 attivi:"
    ps aux | grep -E "ur_robot_driver|ros2|ur_control" | grep -v grep | head -5
else
    echo "❌ Nessun processo ROS2 attivo"
fi
echo ""

# Firewall
echo "6. FIREWALL"
echo "-----------"
if command -v ufw > /dev/null 2>&1; then
    echo "UFW status:"
    sudo ufw status 2>/dev/null | head -5
else
    echo "UFW non installato"
    echo ""
    echo "Verifica iptables:"
    if command -v iptables > /dev/null 2>&1; then
        sudo iptables -L -n | head -10
    else
        echo "iptables non disponibile"
    fi
fi
echo ""

# Test connettività verso ROBOT
echo "7. TEST CONNETTIVITÀ VERSO ROBOT"
echo "---------------------------------"
echo "Test ping a $ROBOT_IP:"
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile"
else
    echo "❌ Robot NON raggiungibile"
fi
echo ""

echo "Test porte ROBOT:"
for PORT in 50002 30001 30002 29999; do
    if timeout 3 nc -zv "$ROBOT_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
        echo "  ✅ Porta $PORT aperta sul robot"
    else
        echo "  ❌ Porta $PORT chiusa o non raggiungibile"
    fi
done
echo ""

# Servizi SSH
echo "8. SERVIZIO SSH"
echo "---------------"
if systemctl is-active --quiet ssh || systemctl is-active --quiet sshd; then
    echo "✅ SSH attivo"
    systemctl status ssh 2>/dev/null | head -3 || systemctl status sshd 2>/dev/null | head -3
else
    echo "❌ SSH NON attivo"
    echo "Avvia con: sudo systemctl start ssh"
fi
echo ""

# Spazio disco
echo "9. SPAZIO DISCO"
echo "---------------"
df -h | head -5
echo ""

# Memoria
echo "10. MEMORIA"
echo "-----------"
free -h
echo ""

echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""

# Verifica prerequisiti
echo "Prerequisiti per External Control:"
if netstat -tuln | grep -q ":50002 "; then
    echo "  ✅ Porta 50002 in ascolto"
else
    echo "  ❌ Porta 50002 NON in ascolto (avvia driver ROS2)"
fi

if pgrep -f "ur_robot_driver" > /dev/null; then
    echo "  ✅ Driver ROS2 attivo"
else
    echo "  ❌ Driver ROS2 NON attivo"
fi

if ping -c 1 -W 1 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "  ✅ Robot raggiungibile"
else
    echo "  ❌ Robot NON raggiungibile"
fi

echo ""
echo "=================================================================================="

