#!/bin/bash

# Script di diagnostica completa per tutte le macchine della rete
# Verifica: ROBOT, AI Accelerator, PC Windows

set +e  # Continua anche con errori

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"
ROBOT_PASS="easybot"

PC_ACCELERATOR_IP="192.168.10.191"
PC_ACCELERATOR_USER="lab"

PC_WINDOWS_IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+' || hostname -I | awk '{print $1}')

echo "=================================================================================="
echo "🔍 DIAGNOSTICA COMPLETA RETE - TUTTE LE MACCHINE"
echo "=================================================================================="
echo ""
echo "Configurazione rete:"
echo "  ROBOT:           $ROBOT_IP (user: $ROBOT_USER)"
echo "  AI Accelerator:  $PC_ACCELERATOR_IP (user: $PC_ACCELERATOR_USER)"
echo "  PC Windows:      $PC_WINDOWS_IP"
echo ""
echo "=================================================================================="
echo ""

# Funzione per testare connessione SSH
test_ssh() {
    local host=$1
    local user=$2
    local pass=$3
    local name=$4
    
    echo "Test connessione SSH a $name ($user@$host)..."
    
    if timeout 5 sshpass -p "$pass" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$user@$host" "echo 'SSH OK'" 2>&1 | grep -q "SSH OK"; then
        echo "  ✅ SSH accessibile"
        return 0
    else
        echo "  ❌ SSH NON accessibile"
        return 1
    fi
}

# Funzione per testare ping
test_ping() {
    local host=$1
    local name=$2
    
    echo "Test ping a $name ($host)..."
    if ping -c 2 -W 2 "$host" > /dev/null 2>&1; then
        echo "  ✅ Raggiungibile via ping"
        return 0
    else
        echo "  ❌ NON raggiungibile via ping"
        return 1
    fi
}

# Funzione per verificare porta
test_port() {
    local host=$1
    local port=$2
    local name=$3
    
    echo "Test porta $port su $name ($host)..."
    if timeout 3 nc -zv "$host" "$port" 2>&1 | grep -q "succeeded\|open"; then
        echo "  ✅ Porta $port aperta"
        return 0
    else
        echo "  ❌ Porta $port chiusa o non raggiungibile"
        return 1
    fi
}

# ================================================================================
# VERIFICA MACCHINA LOCALE (dove gira lo script)
# ================================================================================
echo "1. VERIFICA MACCHINA LOCALE"
echo "---------------------------"
echo "IP locale: $PC_WINDOWS_IP"
echo "Hostname: $(hostname)"
echo "OS: $(uname -a)"
echo ""

# Verifica interfacce di rete
echo "Interfacce di rete:"
ip addr show 2>/dev/null | grep -E "^[0-9]+:|inet " | head -10
echo ""

# Verifica routing
echo "Routing:"
ip route 2>/dev/null | head -5
echo ""

# Verifica porte in ascolto
echo "Porte in ascolto (50001-50004):"
netstat -tuln 2>/dev/null | grep -E "5000[1-4]" || echo "  Nessuna porta 50001-50004 in ascolto"
echo ""

# ================================================================================
# VERIFICA ROBOT
# ================================================================================
echo "2. VERIFICA ROBOT ($ROBOT_IP)"
echo "-----------------------------"

# Test ping
test_ping "$ROBOT_IP" "ROBOT"
PING_ROBOT=$?

# Test SSH
if [ $PING_ROBOT -eq 0 ]; then
    if command -v sshpass > /dev/null 2>&1; then
        test_ssh "$ROBOT_IP" "$ROBOT_USER" "$ROBOT_PASS" "ROBOT"
        SSH_ROBOT=$?
        
        if [ $SSH_ROBOT -eq 0 ]; then
            echo ""
            echo "  Informazioni ROBOT:"
            sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" \
                "echo 'Hostname:' \$(hostname); echo 'IP:' \$(hostname -I); echo 'OS:' \$(uname -a)" 2>&1 | sed 's/^/    /'
            
            echo ""
            echo "  Porte aperte sul ROBOT:"
            sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" \
                "netstat -tuln 2>/dev/null | grep -E '5000[1-4]|30001|30002|29999' || echo 'Nessuna porta rilevante aperta'" 2>&1 | sed 's/^/    /'
        fi
    else
        echo "  ⚠️  sshpass non installato, impossibile testare SSH"
        echo "  Installa: sudo apt-get install sshpass"
    fi
else
    echo "  ⚠️  Robot non raggiungibile, impossibile verificare dettagli"
fi

# Test porte specifiche
if [ $PING_ROBOT -eq 0 ]; then
    echo ""
    echo "  Test porte ROBOT:"
    test_port "$ROBOT_IP" "50002" "ROBOT"
    test_port "$ROBOT_IP" "30001" "ROBOT"
    test_port "$ROBOT_IP" "30002" "ROBOT"
    test_port "$ROBOT_IP" "29999" "ROBOT"
fi

echo ""

# ================================================================================
# VERIFICA AI ACCELERATOR
# ================================================================================
echo "3. VERIFICA AI ACCELERATOR ($PC_ACCELERATOR_IP)"
echo "-----------------------------------------------"

# Test ping
test_ping "$PC_ACCELERATOR_IP" "AI Accelerator"
PING_ACCEL=$?

# Test SSH
if [ $PING_ACCEL -eq 0 ]; then
    echo ""
    echo "  Test SSH (senza password)..."
    if timeout 5 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP" "echo 'SSH OK'" 2>&1 | grep -q "SSH OK"; then
        echo "  ✅ SSH accessibile (senza password)"
        SSH_ACCEL=0
        
        echo ""
        echo "  Informazioni AI Accelerator:"
        ssh -o StrictHostKeyChecking=no "$PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP" \
            "echo 'Hostname:' \$(hostname); echo 'IP:' \$(hostname -I); echo 'OS:' \$(uname -a)" 2>&1 | sed 's/^/    /'
        
        echo ""
        echo "  Porte in ascolto su AI Accelerator:"
        ssh -o StrictHostKeyChecking=no "$PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP" \
            "netstat -tuln 2>/dev/null | grep -E '5000[1-4]|22' || echo 'Nessuna porta rilevante in ascolto'" 2>&1 | sed 's/^/    /'
        
        echo ""
        echo "  Processi ROS2 attivi:"
        ssh -o StrictHostKeyChecking=no "$PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP" \
            "ps aux | grep -E 'ur_robot_driver|ros2|ur_control' | grep -v grep | head -5 || echo 'Nessun processo ROS2 trovato'" 2>&1 | sed 's/^/    /'
        
        echo ""
        echo "  Firewall status:"
        ssh -o StrictHostKeyChecking=no "$PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP" \
            "if command -v ufw > /dev/null 2>&1; then sudo ufw status 2>/dev/null | head -3; else echo 'UFW non installato'; fi" 2>&1 | sed 's/^/    /'
    else
        echo "  ❌ SSH NON accessibile (timeout o autenticazione fallita)"
        SSH_ACCEL=1
    fi
else
    echo "  ⚠️  AI Accelerator non raggiungibile, impossibile verificare dettagli"
    SSH_ACCEL=1
fi

# Test porte specifiche
if [ $PING_ACCEL -eq 0 ]; then
    echo ""
    echo "  Test porte AI Accelerator:"
    test_port "$PC_ACCELERATOR_IP" "22" "AI Accelerator (SSH)"
    test_port "$PC_ACCELERATOR_IP" "50002" "AI Accelerator"
fi

echo ""

# ================================================================================
# VERIFICA CONNETTIVITÀ TRA MACCHINE
# ================================================================================
echo "4. VERIFICA CONNETTIVITÀ TRA MACCHINE"
echo "-------------------------------------"

echo "Da ROBOT a AI Accelerator:"
if [ $PING_ROBOT -eq 0 ] && [ $SSH_ROBOT -eq 0 ]; then
    sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" \
        "ping -c 2 -W 2 $PC_ACCELERATOR_IP 2>&1 | tail -1" 2>&1 | sed 's/^/    /'
else
    echo "  ⚠️  Impossibile testare (robot non raggiungibile)"
fi

echo ""
echo "Da AI Accelerator a ROBOT:"
if [ $PING_ACCEL -eq 0 ] && [ $SSH_ACCEL -eq 0 ]; then
    ssh -o StrictHostKeyChecking=no "$PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP" \
        "ping -c 2 -W 2 $ROBOT_IP 2>&1 | tail -1" 2>&1 | sed 's/^/    /'
else
    echo "  ⚠️  Impossibile testare (AI Accelerator non raggiungibile)"
fi

echo ""

# ================================================================================
# RIEPILOGO FINALE
# ================================================================================
echo "=================================================================================="
echo "📋 RIEPILOGO DIAGNOSTICA"
echo "=================================================================================="
echo ""

echo "ROBOT ($ROBOT_IP):"
if [ $PING_ROBOT -eq 0 ]; then
    echo "  ✅ Raggiungibile via ping"
    if [ $SSH_ROBOT -eq 0 ]; then
        echo "  ✅ SSH accessibile"
    else
        echo "  ❌ SSH NON accessibile"
    fi
else
    echo "  ❌ NON raggiungibile via ping"
fi

echo ""
echo "AI Accelerator ($PC_ACCELERATOR_IP):"
if [ $PING_ACCEL -eq 0 ]; then
    echo "  ✅ Raggiungibile via ping"
    if [ $SSH_ACCEL -eq 0 ]; then
        echo "  ✅ SSH accessibile"
    else
        echo "  ❌ SSH NON accessibile (timeout)"
        echo "  💡 Verifica:"
        echo "     - SSH server attivo: sudo systemctl status ssh"
        echo "     - Firewall non blocca porta 22"
        echo "     - Chiavi SSH configurate"
    fi
else
    echo "  ❌ NON raggiungibile via ping"
    echo "  💡 Verifica connessione di rete fisica"
fi

echo ""
echo "=================================================================================="
echo "🔧 AZIONI CONSIGLIATE"
echo "=================================================================================="
echo ""

if [ $PING_ACCEL -eq 0 ] && [ $SSH_ACCEL -ne 0 ]; then
    echo "1. Verifica SSH su AI Accelerator:"
    echo "   ssh $PC_ACCELERATOR_USER@$PC_ACCELERATOR_IP"
    echo ""
fi

if [ $PING_ROBOT -eq 0 ] && [ $SSH_ROBOT -ne 0 ]; then
    echo "2. Verifica SSH su ROBOT:"
    echo "   sshpass -p '$ROBOT_PASS' ssh $ROBOT_USER@$ROBOT_IP"
    echo ""
fi

echo "3. Per installare sshpass (se mancante):"
echo "   sudo apt-get install sshpass"
echo ""

echo "=================================================================================="

