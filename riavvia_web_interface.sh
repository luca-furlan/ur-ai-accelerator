#!/bin/bash
# Script per riavviare web interface

echo "Riavvio web interface..."

# Ferma tutto
pkill -9 -f web_interface
sleep 2

# Vai nella directory
cd ~/MekoAiAccelerator

# Configura
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
export WEB_HOST=0.0.0.0

# Avvia
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
PID=$!
echo $PID > /tmp/web_interface.pid

sleep 3

# Verifica
if ps -p $PID > /dev/null 2>&1; then
    echo "✅ Web interface avviata (PID: $PID)"
    echo "   URL: http://192.168.10.191:8081"
    echo "   Log: /tmp/web_interface.log"
    
    # Mostra ultime righe del log
    echo ""
    echo "Ultime righe del log:"
    tail -10 /tmp/web_interface.log
else
    echo "❌ Web interface non avviata"
    echo "Log:"
    cat /tmp/web_interface.log
fi




