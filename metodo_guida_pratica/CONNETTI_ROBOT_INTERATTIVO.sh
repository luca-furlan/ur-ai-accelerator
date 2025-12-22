#!/bin/bash

# Script per connettersi interattivamente al robot via SSH
# Permette di eseguire comandi direttamente sul robot

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"
ROBOT_PASS="easybot"

echo "=================================================================================="
echo "🔌 CONNESSIONE INTERATTIVA AL ROBOT"
echo "=================================================================================="
echo ""
echo "Robot IP: $ROBOT_IP"
echo "User: $ROBOT_USER"
echo ""
echo "=================================================================================="
echo ""

# Verifica sshpass
if ! command -v sshpass > /dev/null 2>&1; then
    echo "❌ sshpass non installato"
    echo ""
    echo "Installa: sudo apt-get install sshpass"
    exit 1
fi

# Test connessione
echo "Test connessione..."
if timeout 5 sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_IP" "echo 'OK'" 2>&1 | grep -q "OK"; then
    echo "✅ Connessione OK"
    echo ""
    echo "Connessione al robot..."
    echo "Premi CTRL+D o digita 'exit' per uscire"
    echo ""
    echo "=================================================================================="
    echo ""
    
    # Connetti interattivamente
    sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no "$ROBOT_USER@$ROBOT_IP"
else
    echo "❌ Impossibile connettersi"
    echo ""
    echo "Verifica:"
    echo "  - Robot raggiungibile: ping $ROBOT_IP"
    echo "  - SSH abilitato sul robot"
    echo "  - Password corretta"
    exit 1
fi








