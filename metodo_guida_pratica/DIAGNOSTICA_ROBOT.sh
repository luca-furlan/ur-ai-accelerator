#!/bin/bash

# Script di diagnostica specifica per il ROBOT
# Esegui questo script SUL ROBOT o da una macchina che può accedere al robot

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"
ROBOT_PASS="easybot"

echo "=================================================================================="
echo "🔍 DIAGNOSTICA ROBOT ($ROBOT_IP)"
echo "=================================================================================="
echo ""

# Verifica se sshpass è installato
if ! command -v sshpass > /dev/null 2>&1; then
    echo "⚠️  sshpass non installato"
    echo "Installa con: sudo apt-get install sshpass"
    echo ""
    exit 1
fi

# Test connessione
echo "1. TEST CONNESSIONE"
echo "------------------"
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile via ping"
else
    echo "❌ Robot NON raggiungibile via ping"
    exit 1
fi

# Informazioni sistema
echo ""
echo "2. INFORMAZIONI SISTEMA"
echo "----------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Hostname: $(hostname)"
echo "IP: $(hostname -I)"
echo "OS: $(uname -a)"
echo ""
echo "Interfacce di rete:"
ip addr show | grep -E "^[0-9]+:|inet " | head -10
EOF

# Porte aperte
echo ""
echo "3. PORTE APERTE"
echo "---------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Porte rilevanti:"
netstat -tuln 2>/dev/null | grep -E "5000[1-4]|30001|30002|29999|22" || echo "Nessuna porta rilevante trovata"
EOF

# Processi attivi
echo ""
echo "4. PROCESSI ATTIVI"
echo "------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Processi URCap/External Control:"
ps aux | grep -iE "urcap|external|control" | grep -v grep || echo "Nessun processo trovato"
EOF

# Configurazione rete
echo ""
echo "5. CONFIGURAZIONE RETE"
echo "---------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Routing:"
ip route | head -5
echo ""
echo "DNS:"
cat /etc/resolv.conf 2>/dev/null | grep nameserver || echo "Nessun DNS configurato"
EOF

# Test connettività verso AI Accelerator
echo ""
echo "6. TEST CONNETTIVITÀ VERSO AI ACCELERATOR"
echo "------------------------------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
PC_IP="192.168.10.191"
echo "Test ping a $PC_IP:"
if ping -c 2 -W 2 "$PC_IP" > /dev/null 2>&1; then
    echo "✅ AI Accelerator raggiungibile"
else
    echo "❌ AI Accelerator NON raggiungibile"
fi
echo ""
echo "Test porta 50002 su $PC_IP:"
if timeout 3 nc -zv "$PC_IP" 50002 2>&1 | grep -q "succeeded\|open"; then
    echo "✅ Porta 50002 aperta su AI Accelerator"
else
    echo "❌ Porta 50002 chiusa o non raggiungibile"
fi
EOF

echo ""
echo "=================================================================================="

