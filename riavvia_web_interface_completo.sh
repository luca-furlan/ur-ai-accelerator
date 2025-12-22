#!/bin/bash
# Script completo per riavviare web interface con tutte le nuove funzionalità

cd ~/MekoAiAccelerator || exit 1

echo "=========================================="
echo "RIAVVIO WEB INTERFACE COMPLETO"
echo "=========================================="
echo ""

# 1. Ferma processi esistenti
echo "[1/5] Fermo processi esistenti..."
pkill -f web_interface
pkill -f "python.*web_interface"
sleep 2
echo "✅ Processi fermati"
echo ""

# 2. Verifica che non ci siano processi residui
echo "[2/5] Verifica processi residui..."
if pgrep -f web_interface > /dev/null; then
    echo "⚠️  Ancora processi attivi, forzo kill..."
    pkill -9 -f web_interface
    sleep 1
fi
echo "✅ Nessun processo residuo"
echo ""

# 3. Source ROS2 environment
echo "[3/5] Configurazione ROS2 environment..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# CRITICAL: Ensure LD_LIBRARY_PATH includes ROS2 lib directory
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}

# Also ensure PYTHONPATH is set correctly
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}

echo "✅ ROS2 environment configurato"
echo "   ROS_DISTRO=${ROS_DISTRO}"
echo "   LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
echo ""

# 4. Verifica ROS2 disponibilità
echo "[4/5] Verifica ROS2 disponibilità..."
if python3 -c "import rclpy; print('✅ rclpy disponibile')" 2>&1; then
    echo "✅ rclpy disponibile"
else
    echo "⚠️  rclpy non disponibile - alcune funzionalità potrebbero non funzionare"
    echo "   Suggerimento: verifica installazione ROS2"
fi
echo ""

# 5. Configura variabili ambiente
echo "[5/5] Configurazione variabili ambiente..."
export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}
export WEB_HOST=${WEB_HOST:-0.0.0.0}
export WEB_PORT=${WEB_PORT:-8080}

echo "✅ Variabili configurate:"
echo "   UR_ROBOT_IP=${UR_ROBOT_IP}"
echo "   WEB_HOST=${WEB_HOST}"
echo "   WEB_PORT=${WEB_PORT}"
echo ""

# 6. Avvia web interface
echo "=========================================="
echo "AVVIO WEB INTERFACE"
echo "=========================================="
echo ""
echo "🌐 Web interface sarà disponibile su:"
echo "   http://${WEB_HOST}:${WEB_PORT}"
echo ""
echo "📋 Nuove funzionalità disponibili:"
echo "   ✅ Sezione Orbbec Camera"
echo "   ✅ Sezione MoveIt2 Motion Planning"
echo "   ✅ Object Detections"
echo "   ✅ Controller ottimizzato (8ms delay, 125Hz)"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

# 6. Avvia watchdog (se non già attivo)
echo "[6/6] Verifica watchdog..."
if ! pgrep -f "watchdog_web_interface" > /dev/null; then
    echo "Avvio watchdog per monitoraggio automatico..."
    nohup bash watchdog_web_interface.sh > /tmp/watchdog.log 2>&1 &
    echo "✅ Watchdog avviato"
else
    echo "✅ Watchdog già attivo"
fi
echo ""

# 7. Avvia web interface
echo "=========================================="
echo "AVVIO WEB INTERFACE"
echo "=========================================="
echo ""
echo "🌐 Web interface sarà disponibile su:"
echo "   http://${WEB_HOST}:${WEB_PORT}"
echo ""
echo "📋 Nuove funzionalità disponibili:"
echo "   ✅ Sezione Orbbec Camera"
echo "   ✅ Sezione MoveIt2 Motion Planning"
echo "   ✅ Object Detections"
echo "   ✅ Controller ottimizzato (8ms delay, 125Hz)"
echo "   ✅ Auto-restart all'accesso pagina"
echo "   ✅ Health check automatico"
echo "   ✅ Watchdog per monitoraggio"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

# Avvia web interface
python3 -m remote_ur_control.web_interface
