#!/bin/bash

# Script per verificare robot senza sshpass (usa solo comandi disponibili)

ROBOT_IP="192.168.10.194"
PC_IP="192.168.10.191"

echo "=================================================================================="
echo "🔍 VERIFICA ROBOT (SENZA SSH - SOLO RETE)"
echo "=================================================================================="
echo ""
echo "Robot IP: $ROBOT_IP"
echo "PC IP: $PC_IP"
echo ""
echo "=================================================================================="
echo ""

# 1. Test ping
echo "1. TEST PING ROBOT"
echo "------------------"
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile via ping"
else
    echo "❌ Robot NON raggiungibile via ping"
    exit 1
fi
echo ""

# 2. Test porte robot
echo "2. TEST PORTE ROBOT"
echo "-------------------"
for PORT in 22 29999 30001 30002 50002; do
    echo -n "   Porta $PORT: "
    if timeout 3 nc -zv "$ROBOT_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
        echo "✅ APERTA"
    else
        echo "❌ CHIUSA o non raggiungibile"
    fi
done
echo ""

# 3. Test connettività robot verso PC
echo "3. TEST CONNETTIVITÀ ROBOT → PC"
echo "-------------------------------"
echo "   (Verifica se robot può raggiungere PC sulla porta 50002)"
echo ""
echo "   ⚠️  Per verificare completamente, connettiti al robot via SSH:"
echo "      ssh root@$ROBOT_IP"
echo "      (password: easybot)"
echo ""
echo "   Poi esegui sul robot:"
echo "      ping -c 3 $PC_IP"
echo "      nc -zv $PC_IP 50002"
echo ""

# 4. Verifica porta 50002 su PC
echo "4. VERIFICA PORTA 50002 SU PC"
echo "-----------------------------"
if netstat -tuln | grep -q ":50002 "; then
    echo "✅ PC in ascolto sulla porta 50002"
    netstat -tuln | grep ":50002 "
else
    echo "❌ PC NON in ascolto sulla porta 50002"
    echo ""
    echo "   💡 SOLUZIONE:"
    echo "   Avvia driver ROS2: ./START_RAPIDO.sh"
fi
echo ""

# 5. Verifica driver ROS2
echo "5. VERIFICA DRIVER ROS2"
echo "----------------------"
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "✅ Driver ROS2 attivo"
    ps aux | grep -E "ur_robot_driver|ur_control.launch" | grep -v grep | head -3
else
    echo "❌ Driver ROS2 NON attivo"
fi
echo ""

# 6. Istruzioni per SSH
echo "6. CONNESSIONE SSH AL ROBOT"
echo "---------------------------"
echo ""
echo "Per connettersi al robot e verificare configurazione:"
echo ""
echo "OPZIONE 1: Con sshpass (se installato)"
echo "  sshpass -p 'easybot' ssh root@$ROBOT_IP"
echo ""
echo "OPZIONE 2: SSH normale (richiede password)"
echo "  ssh root@$ROBOT_IP"
echo "  Password: easybot"
echo ""
echo "OPZIONE 3: Usa script interattivo"
echo "  ./CONNETTI_ROBOT_INTERATTIVO.sh"
echo ""

# 7. Comandi da eseguire sul robot
echo "7. COMANDI DA ESEGUIRE SUL ROBOT (dopo connessione SSH)"
echo "-------------------------------------------------------"
echo ""
echo "Dopo esserti connesso al robot, esegui:"
echo ""
echo "# Verifica porte aperte"
echo "netstat -tuln | grep -E '5000[1-4]|30001|30002|29999'"
echo ""
echo "# Verifica processi"
echo "ps aux | grep -iE 'urcap|external|control'"
echo ""
echo "# Test connettività verso PC"
echo "ping -c 3 $PC_IP"
echo "nc -zv $PC_IP 50002"
echo ""
echo "# Informazioni sistema"
echo "hostname"
echo "hostname -I"
echo "uname -a"
echo ""

echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "✅ Verifiche di rete completate"
echo ""
echo "⚠️  VERIFICHE MANUALI RICHIESTE:"
echo ""
echo "1. Connettiti al robot via SSH per verifiche complete"
echo "2. Sul Teach Pendant:"
echo "   - EtherNet/IP DISABILITATO (Installation → Fieldbus)"
echo "   - PROFINET DISABILITATO (Installation → Fieldbus)"
echo "   - Remote Control abilitato (Settings → System → Remote Control)"
echo "   - External Control configurato: Host IP = $PC_IP, Port = 50002"
echo ""
echo "=================================================================================="








