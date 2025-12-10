#!/bin/bash
# Script per avviare web interface in modo affidabile

AI_ACCELERATOR="lab@192.168.10.191"

echo "Avvio web interface su AI Accelerator..."

ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR << 'ENDSSH'
cd ~/MekoAiAccelerator

# Ferma eventuali istanze precedenti
pkill -f "web_interface" 2>/dev/null
sleep 2

# Verifica che il file esista
if [ ! -f "remote_ur_control/web_interface.py" ]; then
    echo "ERRORE: web_interface.py non trovato!"
    exit 1
fi

# Configura variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
export WEB_HOST=0.0.0.0

# Avvia in background
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
PID=$!
echo $PID > /tmp/web_interface.pid

# Aspetta che si avvii
sleep 5

# Verifica che sia in esecuzione
if ps -p $PID > /dev/null; then
    echo "✅ Web interface avviata (PID: $PID)"
    echo "   URL: http://192.168.10.191:8081"
    echo "   Log: /tmp/web_interface.log"
else
    echo "❌ Web interface non avviata correttamente"
    echo "   Controlla log: /tmp/web_interface.log"
    tail -20 /tmp/web_interface.log
    exit 1
fi
ENDSSH

echo ""
echo "Verifica connessione..."
sleep 2

# Test connessione
if curl -s http://192.168.10.191:8081/ > /dev/null 2>&1; then
    echo "✅ Web interface raggiungibile!"
    echo ""
    echo "Apri il browser su: http://192.168.10.191:8081"
else
    echo "⚠️  Web interface potrebbe non essere ancora pronta"
    echo "   Attendi qualche secondo e riprova"
fi




