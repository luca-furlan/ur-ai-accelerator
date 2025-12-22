#!/bin/bash
# Script per avviare il sistema vision completo (Camera + YOLO)

cd ~/MekoAiAccelerator || exit 1

echo "=========================================="
echo "AVVIO SISTEMA VISION COMPLETO"
echo "=========================================="
echo ""

# Source ROS2
echo "[1/3] Configurazione ROS2 environment..."
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}

echo "✅ ROS2 environment configurato"
echo ""

# Verifica camera
echo "[2/3] Verifica camera Orbbec..."
if ! pgrep -f orbbec_camera > /dev/null; then
    echo "⚠️  Camera Orbbec non attiva"
    echo "   Avvia camera con: ros2 launch orbbec_camera gemini_330_series.launch.py"
    echo "   Oppure usa il pulsante 'Start Camera' nella web interface"
else
    echo "✅ Camera Orbbec attiva"
fi
echo ""

# Avvia YOLO detector
echo "[3/3] Avvio YOLO detector..."
if pgrep -f vision_yolo_detector > /dev/null; then
    echo "⚠️  YOLO detector già attivo"
    echo "   PID: $(pgrep -f vision_yolo_detector)"
else
    echo "Avvio vision_yolo_detector.py..."
    nohup python3 vision_yolo_detector.py > /tmp/yolo_detector.log 2>&1 &
    YOLO_PID=$!
    sleep 2
    
    if ps -p $YOLO_PID > /dev/null; then
        echo "✅ YOLO detector avviato (PID: $YOLO_PID)"
        echo "   Log: tail -f /tmp/yolo_detector.log"
    else
        echo "❌ Errore avvio YOLO detector"
        echo "   Controlla log: cat /tmp/yolo_detector.log"
        exit 1
    fi
fi
echo ""

echo "=========================================="
echo "✅ SISTEMA VISION AVVIATO"
echo "=========================================="
echo ""
echo "Componenti attivi:"
echo "  - Camera Orbbec: $(pgrep -f orbbec_camera > /dev/null && echo '✅ Attiva' || echo '❌ Non attiva')"
echo "  - YOLO Detector: $(pgrep -f vision_yolo_detector > /dev/null && echo '✅ Attivo' || echo '❌ Non attivo')"
echo ""
echo "Topics ROS2:"
echo "  - /vision/detections_3d (detections con coordinate 3D)"
echo "  - /vision/annotated_image (immagine con bounding boxes)"
echo ""
echo "Nella web interface:"
echo "  1. Clicca 'Start Vision System' per inizializzare subscriber"
echo "  2. I pezzi rilevati appariranno nella sezione 'Object Detections'"
echo "  3. Clicca 'Avvicinati al Pezzo' per muovere il robot"
echo ""
