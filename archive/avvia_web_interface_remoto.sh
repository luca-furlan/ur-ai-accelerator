#!/bin/bash
# Script per avviare web interface in background

ssh lab@192.168.10.191 << 'ENDSSH'
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081

# Verifica se già in esecuzione
if pgrep -f "web_interface" > /dev/null; then
    echo "Web interface già in esecuzione"
    exit 0
fi

# Avvia in background
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
echo $! > /tmp/web_interface.pid
echo "Web interface avviata (PID: $(cat /tmp/web_interface.pid))"
sleep 2
ENDSSH










