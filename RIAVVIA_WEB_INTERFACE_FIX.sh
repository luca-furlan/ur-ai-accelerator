#!/bin/bash
# Script per riavviare web interface con fix socket

echo "================================================================================"
echo "RIAVVIO WEB INTERFACE CON FIX SOCKET"
echo "================================================================================"
echo

cd ~/MekoAiAccelerator || exit 1

# Ferma web interface esistente
echo "1. Fermo web interface esistente..."
pkill -9 -f web_interface
sleep 2
echo "   ✅ Web interface fermata"
echo

# Verifica che il file sia aggiornato
echo "2. Verifica file aggiornato..."
if grep -q "USA SEMPRE SOCKET DIRETTO" remote_ur_control/web_interface.py; then
    echo "   ✅ File contiene fix socket"
else
    echo "   ❌ File NON contiene fix socket!"
    echo "   💡 Copia il file aggiornato dalla macchina Windows"
    exit 1
fi
echo

# Configura variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
export WEB_HOST=0.0.0.0

# Avvia web interface
echo "3. Avvio web interface..."
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
PID=$!
echo $PID > /tmp/web_interface.pid

sleep 3

# Verifica
if ps -p $PID > /dev/null 2>&1; then
    echo "   ✅ Web interface avviata (PID: $PID)"
    echo "   URL: http://192.168.10.191:8081"
    echo "   Log: /tmp/web_interface.log"
    echo
    echo "   Ultime righe del log:"
    tail -10 /tmp/web_interface.log
else
    echo "   ❌ Web interface non avviata"
    echo "   Log completo:"
    cat /tmp/web_interface.log
    exit 1
fi

echo
echo "================================================================================"
echo "✅ WEB INTERFACE RIAVVIATA!"
echo "================================================================================"
echo
echo "Ora prova a muovere il joystick nella web interface."
echo "Dovresti vedere 'Socket control (servoj_velocity)' invece di 'ROS2 speedj'"
echo




