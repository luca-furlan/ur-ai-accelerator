#!/bin/bash
# Script completo per applicare fix RTDE e riavviare web interface

AI_ACCELERATOR="lab@192.168.10.191"
REMOTE_DIR="~/MekoAiAccelerator"

echo "=================================================================================="
echo "APPLICAZIONE FIX RTDE E RIAVVIO WEB INTERFACE"
echo "=================================================================================="
echo ""

# 1. Copia file
echo "1. Copia file aggiornato..."
scp -o StrictHostKeyChecking=no remote_ur_control/web_interface.py ${AI_ACCELERATOR}:${REMOTE_DIR}/remote_ur_control/web_interface.py
if [ $? -eq 0 ]; then
    echo "   ✅ File copiato"
else
    echo "   ❌ Errore copia file"
    exit 1
fi

echo ""

# 2. Ferma web interface esistente
echo "2. Ferma web interface esistente..."
ssh -o StrictHostKeyChecking=no ${AI_ACCELERATOR} "pkill -f web_interface; sleep 2"
echo "   ✅ Processi fermati"

echo ""

# 3. Avvia web interface
echo "3. Avvia web interface..."
ssh -o StrictHostKeyChecking=no ${AI_ACCELERATOR} << 'ENDSSH'
cd ~/MekoAiAccelerator
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
if ps -p $PID > /dev/null 2>&1; then
    echo "✅ Web interface avviata (PID: $PID)"
    echo "   URL: http://192.168.10.191:8081"
else
    echo "❌ Web interface non avviata correttamente"
    echo "   Log:"
    tail -20 /tmp/web_interface.log
    exit 1
fi
ENDSSH

if [ $? -eq 0 ]; then
    echo "   ✅ Web interface avviata"
else
    echo "   ❌ Errore avvio web interface"
    exit 1
fi

echo ""

# 4. Verifica
echo "4. Verifica connessione..."
sleep 3

if curl -s http://192.168.10.191:8081/ > /dev/null 2>&1; then
    echo "   ✅ Web interface raggiungibile"
else
    echo "   ⚠️  Web interface potrebbe non essere ancora pronta"
    echo "   Attendi qualche secondo e ricarica la pagina"
fi

echo ""
echo "=================================================================================="
echo "✅ FIX APPLICATO E WEB INTERFACE RIAVVIATA"
echo "=================================================================================="
echo ""
echo "Apri il browser su: http://192.168.10.191:8081"
echo ""
echo "Dovresti vedere:"
echo "  ✅ Joint Positions: [valori aggiornati ogni 2 secondi]"
echo "  ✅ TCP Pose: [valori aggiornati ogni 2 secondi]"
echo "  ❌ Nessun errore RTDE"
echo ""




