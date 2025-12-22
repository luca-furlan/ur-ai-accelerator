#!/bin/bash
# Script per diagnosticare e riavviare la web interface
# Da eseguire SULL'AI ACCELERATOR

echo "=========================================="
echo "DIAGNOSTICA E FIX WEB INTERFACE"
echo "=========================================="
echo ""

# 1. Ferma processi esistenti
echo "1. Fermo processi esistenti..."
pkill -f "web_interface" 2>/dev/null || true
sleep 2

# 2. Verifica file
echo ""
echo "2. Verifica file..."
cd ~/MekoAiAccelerator
if [ ! -f remote_ur_control/web_interface.py ]; then
    echo "❌ web_interface.py non trovato!"
    exit 1
fi
echo "✅ File presente"

# 3. Verifica dipendenze
echo ""
echo "3. Verifica dipendenze..."
if ! python3 -c "import flask" 2>/dev/null; then
    echo "⚠️  Flask non trovato - installazione..."
    pip3 install --user flask flask-cors
fi
echo "✅ Flask OK"

# 4. Verifica connessione robot
echo ""
echo "4. Verifica robot..."
export UR_ROBOT_IP=192.168.10.194
if ping -c 1 $UR_ROBOT_IP > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile"
else
    echo "❌ Robot NON raggiungibile"
fi

# 5. Source ROS2
echo ""
echo "5. Setup ROS2..."
source /opt/ros/humble/setup.bash 2>/dev/null || echo "⚠️  ROS2 non trovato (userà socket fallback)"
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# 6. Configura variabili
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# 7. Verifica porta
echo ""
echo "6. Verifica porta 8080..."
if lsof -i :8080 2>/dev/null | grep -q LISTEN; then
    echo "⚠️  Porta 8080 occupata - liberazione..."
    fuser -k 8080/tcp 2>/dev/null || true
    sleep 2
fi

# 8. Avvia web interface
echo ""
echo "7. Avvio web interface..."
cd ~/MekoAiAccelerator
python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
WEB_PID=$!
sleep 3

# 9. Verifica avvio
if ps -p $WEB_PID > /dev/null 2>&1; then
    echo "✅ Web interface avviata (PID: $WEB_PID)"
    echo ""
    echo "=========================================="
    echo "✅ WEB INTERFACE ATTIVA!"
    echo "=========================================="
    echo ""
    echo "🌐 Connettiti qui:"
    echo "   http://$(hostname -I | awk '{print $1}'):8080"
    echo "   oppure"
    echo "   http://192.168.10.191:8080"
    echo ""
    echo "📋 Log disponibili in: /tmp/web_interface.log"
    echo "   Per vedere i log: tail -f /tmp/web_interface.log"
    echo ""
else
    echo "❌ Errore avvio web interface"
    echo ""
    echo "📋 Controlla i log:"
    cat /tmp/web_interface.log
    exit 1
fi

