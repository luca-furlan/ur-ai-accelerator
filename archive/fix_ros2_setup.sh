#!/bin/bash
# Fix ROS2 setup per script avvio

cd ~/MekoAiAccelerator

# Fix script avvio - gestione ROS2 migliore
cat > avvia_vision_completo.sh << 'EOFBASH'
#!/bin/bash

# Source ROS2 solo se disponibile
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    
    # Verifica ROS2 funzionante
    if command -v ros2 &> /dev/null; then
        echo "✅ ROS2 configurato"
    else
        echo "⚠️  ROS2 non disponibile nel PATH (continua comunque)"
    fi
else
    echo "⚠️  ROS2 Humble non trovato"
fi

[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

export PYTHONPATH=$(pwd):$PYTHONPATH
export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}

echo "🚀 Avvio Vision System Completo"
echo ""

# Avvia camera solo se ROS2 funziona e driver disponibile
if command -v ros2 &> /dev/null && ros2 pkg list 2>/dev/null | grep -q orbbec_camera; then
    echo "→ Avvio camera..."
    ros2 launch orbbec_camera gemini_330_series.launch.py > /tmp/camera.log 2>&1 &
    sleep 3
else
    echo "⚠️  Camera non disponibile (skip)"
fi

# Avvia vision detector (funziona anche senza ROS2 completo)
echo "→ Avvio vision detector..."
python3 vision_yolo_detector.py > /tmp/vision.log 2>&1 &
DETECTOR_PID=$!
sleep 2

# Avvia coordinator (funziona anche senza ROS2 completo)
echo "→ Avvio coordinator..."
python3 vision_robot_coordinator.py > /tmp/coordinator.log 2>&1 &
COORDINATOR_PID=$!
sleep 1

echo ""
echo "✅ Vision system avviato!"
echo "  Detector PID: $DETECTOR_PID"
echo "  Coordinator PID: $COORDINATOR_PID"
echo ""
echo "Log: /tmp/vision.log, /tmp/coordinator.log"
echo ""

# Gestione porta occupata - AUTO-FIX
WEB_PORT=${WEB_PORT:-8080}
echo "→ Verifica e liberazione porta $WEB_PORT..."

# Kill tutti i processi Python web_interface esistenti
pkill -f "python.*web_interface" 2>/dev/null || true
sleep 1

# Kill processo sulla porta specifica
PORT_PID=$(lsof -ti :$WEB_PORT 2>/dev/null | head -1)
if [ -n "$PORT_PID" ]; then
    echo "⚠️  Porta $WEB_PORT occupata da PID $PORT_PID"
    kill -TERM $PORT_PID 2>/dev/null || true
    sleep 2
    kill -9 $PORT_PID 2>/dev/null || true
    sleep 1
fi

# Verifica finale
if lsof -ti :$WEB_PORT >/dev/null 2>&1; then
    echo "⚠️  Porta ancora occupata, uso porta alternativa 8081"
    WEB_PORT=8081
    export WEB_PORT
else
    echo "✅ Porta $WEB_PORT libera"
fi

echo ""
echo "Avvio web interface su porta $WEB_PORT..."
echo ""

export WEB_PORT
python3 -m remote_ur_control.web_interface
EOFBASH

chmod +x avvia_vision_completo.sh
echo "✅ Script avvio aggiornato"

