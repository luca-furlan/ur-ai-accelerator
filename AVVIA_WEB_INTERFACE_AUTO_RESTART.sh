#!/bin/bash
# Script completo per avviare web interface con auto-restart e watchdog

cd ~/MekoAiAccelerator || exit 1

echo "=========================================="
echo "AVVIO WEB INTERFACE CON AUTO-RESTART"
echo "=========================================="
echo ""

# 1. Ferma tutto
echo "[1/6] Fermo processi esistenti..."
pkill -f web_interface 2>/dev/null
pkill -f watchdog_web_interface 2>/dev/null
sleep 2
echo "✅ Processi fermati"
echo ""

# 2. Source ROS2
echo "[2/6] Configurazione ROS2..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}
export WEB_HOST=${WEB_HOST:-0.0.0.0}
export WEB_PORT=${WEB_PORT:-8080}

echo "✅ ROS2 configurato"
echo ""

# 3. Rendi eseguibili gli script
echo "[3/6] Preparazione script..."
chmod +x watchdog_web_interface.sh
chmod +x riavvia_web_interface_completo.sh
echo "✅ Script pronti"
echo ""

# 4. Avvia watchdog
echo "[4/6] Avvio watchdog..."
nohup bash watchdog_web_interface.sh > /tmp/watchdog.log 2>&1 &
WATCHDOG_PID=$!
echo "✅ Watchdog avviato (PID: $WATCHDOG_PID)"
echo ""

# 5. Attendi che watchdog si avvii
sleep 2

# 6. Avvia web interface
echo "[5/6] Avvio web interface..."
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
WEB_PID=$!
echo "$WEB_PID" > /tmp/web_interface.pid
echo "✅ Web interface avviato (PID: $WEB_PID)"
echo ""

# 7. Verifica
echo "[6/6] Verifica avvio..."
sleep 3

if pgrep -f "web_interface" > /dev/null && pgrep -f "watchdog_web_interface" > /dev/null; then
    echo "=========================================="
    echo "✅ SISTEMA AVVIATO CORRETTAMENTE"
    echo "=========================================="
    echo ""
    echo "🌐 Web interface: http://${WEB_HOST}:${WEB_PORT}"
    echo "📊 Watchdog: monitoraggio attivo"
    echo ""
    echo "📋 Funzionalità:"
    echo "   ✅ Auto-restart all'accesso pagina"
    echo "   ✅ Health check ogni 10 secondi"
    echo "   ✅ Watchdog monitoraggio ogni 15 secondi"
    echo "   ✅ Auto-recovery se processo si blocca"
    echo ""
    echo "📝 Log:"
    echo "   Web interface: tail -f /tmp/web_interface.log"
    echo "   Watchdog: tail -f /tmp/web_interface_watchdog.log"
    echo ""
    echo "Per fermare:"
    echo "   pkill -f web_interface"
    echo "   pkill -f watchdog_web_interface"
    echo "=========================================="
else
    echo "⚠️  Errore avvio - controlla i log"
    echo "   tail -f /tmp/web_interface.log"
    echo "   tail -f /tmp/watchdog.log"
fi
