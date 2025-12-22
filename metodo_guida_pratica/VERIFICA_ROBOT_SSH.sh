#!/bin/bash

# Script per connettersi al robot via SSH e verificare configurazione
# Verifica EtherNet/IP, Remote Control, porte, etc.

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"
ROBOT_PASS="easybot"

echo "=================================================================================="
echo "🔍 VERIFICA CONFIGURAZIONE ROBOT VIA SSH"
echo "=================================================================================="
echo ""
echo "Robot IP: $ROBOT_IP"
echo "User: $ROBOT_USER"
echo ""
echo "=================================================================================="
echo ""

# Verifica sshpass installato
if ! command -v sshpass > /dev/null 2>&1; then
    echo "❌ sshpass non installato"
    echo ""
    echo "Installa sshpass:"
    echo "  sudo apt-get install sshpass"
    echo ""
    exit 1
fi

# Test connessione
echo "1. TEST CONNESSIONE SSH"
echo "----------------------"
if timeout 5 sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_IP" "echo 'SSH OK'" 2>&1 | grep -q "SSH OK"; then
    echo "✅ SSH accessibile"
else
    echo "❌ SSH NON accessibile"
    echo ""
    echo "Possibili cause:"
    echo "  - Robot non raggiungibile via rete"
    echo "  - SSH non abilitato sul robot"
    echo "  - Password errata"
    exit 1
fi
echo ""

# Informazioni sistema
echo "2. INFORMAZIONI SISTEMA ROBOT"
echo "-----------------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Hostname: $(hostname)"
echo "IP: $(hostname -I)"
echo "OS: $(uname -a)"
echo ""
echo "Versione Polyscope:"
cat /etc/polyscope/version 2>/dev/null || echo "Versione non trovata"
EOF
echo ""

# Verifica EtherNet/IP e PROFINET
echo "3. VERIFICA ETHERNET/IP E PROFINET (IMPORTANTE!)"
echo "------------------------------------------------"
echo ""
echo "⚠️  Questa verifica richiede accesso al Teach Pendant"
echo "   Non posso verificare via SSH, devi controllare manualmente:"
echo ""
echo "   Sul Teach Pendant:"
echo "   1. Vai su: Installation → Fieldbus"
echo "   2. Verifica:"
echo "      - EtherNet/IP deve essere DISABILITATO ❌"
echo "      - PROFINET deve essere DISABILITATO ❌"
echo "      - Solo Ethernet normale può essere abilitato ✅"
echo ""
echo "   Questo è la causa più comune di segmentation fault (Issue #31)!"
echo ""

# Verifica Remote Control
echo "4. VERIFICA REMOTE CONTROL"
echo "-------------------------"
echo ""
echo "⚠️  Questa verifica richiede accesso al Teach Pendant"
echo "   Controlla manualmente:"
echo ""
echo "   Sul Teach Pendant:"
echo "   1. Vai su: Settings → System → Remote Control"
echo "   2. Deve essere 'Enabled'"
echo ""

# Verifica porte aperte sul robot
echo "5. PORTE APERTE SUL ROBOT"
echo "-------------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Porte rilevanti (50002, 30001, 30002, 29999, 22):"
netstat -tuln 2>/dev/null | grep -E "5000[1-4]|30001|30002|29999|22" || echo "Nessuna porta rilevante trovata"
EOF
echo ""

# Verifica processi attivi
echo "6. PROCESSI ATTIVI"
echo "------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Processi URCap/External Control:"
ps aux | grep -iE "urcap|external|control" | grep -v grep || echo "Nessun processo trovato"
EOF
echo ""

# Verifica configurazione rete
echo "7. CONFIGURAZIONE RETE"
echo "---------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Interfacce di rete:"
ip addr show | grep -E "^[0-9]+:|inet " | head -10
echo ""
echo "Routing:"
ip route | head -5
EOF
echo ""

# Test connettività verso PC
echo "8. TEST CONNETTIVITÀ VERSO PC (192.168.10.191)"
echo "----------------------------------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
PC_IP="192.168.10.191"
echo "Test ping a $PC_IP:"
if ping -c 2 -W 2 "$PC_IP" > /dev/null 2>&1; then
    echo "  ✅ PC raggiungibile"
else
    echo "  ❌ PC NON raggiungibile"
fi
echo ""
echo "Test porta 50002 su $PC_IP:"
if command -v nc > /dev/null 2>&1; then
    if timeout 3 nc -zv "$PC_IP" 50002 2>&1 | grep -q "succeeded\|open"; then
        echo "  ✅ Porta 50002 aperta su PC"
    else
        echo "  ❌ Porta 50002 chiusa o non raggiungibile su PC"
    fi
else
    echo "  ⚠️  nc (netcat) non disponibile sul robot"
fi
EOF
echo ""

# Verifica programmi sul robot
echo "9. PROGRAMMI SUL ROBOT"
echo "---------------------"
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP" << 'EOF'
echo "Programmi nella directory programmi:"
ls -la /programs/ 2>/dev/null | head -10 || echo "Directory programmi non accessibile"
EOF
echo ""

# Riepilogo
echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "✅ Verifiche completate via SSH"
echo ""
echo "⚠️  VERIFICHE MANUALI RICHIESTE SUL TEACH PENDANT:"
echo ""
echo "1. EtherNet/IP DISABILITATO (Installation → Fieldbus)"
echo "   - Questa è la causa più comune di segmentation fault!"
echo ""
echo "2. PROFINET DISABILITATO (Installation → Fieldbus)"
echo ""
echo "3. Remote Control abilitato (Settings → System → Remote Control)"
echo ""
echo "4. Programma External Control configurato:"
echo "   - Host IP: 192.168.10.191"
echo "   - Port: 50002"
echo "   - Programma in PLAYING"
echo ""
echo "=================================================================================="
echo ""
echo "Per connettersi manualmente al robot:"
echo "  sshpass -p '$ROBOT_PASS' ssh $ROBOT_USER@$ROBOT_IP"
echo ""








