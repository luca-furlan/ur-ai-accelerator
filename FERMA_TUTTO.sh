#!/bin/bash
# Script per FERMARE COMPLETAMENTE web interface e watchdog

cd ~/MekoAiAccelerator || exit 1

echo "=========================================="
echo "FERMATA COMPLETA WEB INTERFACE"
echo "=========================================="
echo ""

# 1. Ferma processi vision gestiti dalla web interface
echo "[1/5] Fermo processi vision..."
if pgrep -f orbbec_camera > /dev/null; then
    pkill -f orbbec_camera
    echo "✅ Camera Orbbec fermata"
else
    echo "ℹ️  Camera Orbbec non attiva"
fi

if pgrep -f vision_yolo_detector > /dev/null; then
    pkill -f vision_yolo_detector
    echo "✅ YOLO detector fermato"
else
    echo "ℹ️  YOLO detector non attivo"
fi

if pgrep -f moveit.launch.py > /dev/null; then
    pkill -f moveit.launch.py
    echo "✅ MoveIt fermato"
else
    echo "ℹ️  MoveIt non attivo"
fi
echo ""

# 2. Ferma watchdog PRIMA di tutto (IMPORTANTE!)
echo "[2/5] Fermo watchdog..."
pkill -f watchdog_web_interface
pkill -f "bash.*watchdog_web_interface"
sleep 2

# Verifica che watchdog sia fermato
if pgrep -f watchdog_web_interface > /dev/null; then
    echo "⚠️  Watchdog ancora attivo, forzo kill..."
    pkill -9 -f watchdog_web_interface
    sleep 1
fi

if pgrep -f watchdog_web_interface > /dev/null; then
    echo "❌ ERRORE: Watchdog non si ferma!"
    echo "   Processi watchdog:"
    pgrep -af watchdog_web_interface
else
    echo "✅ Watchdog fermato"
fi
echo ""

# 3. Ferma web interface
echo "[3/5] Fermo web interface..."
pkill -f web_interface
pkill -f "python.*web_interface"
sleep 2

# Verifica che web interface sia fermato
if pgrep -f web_interface > /dev/null; then
    echo "⚠️  Web interface ancora attivo, forzo kill..."
    pkill -9 -f web_interface
    sleep 1
fi

if pgrep -f web_interface > /dev/null; then
    echo "❌ ERRORE: Web interface non si ferma!"
    echo "   Processi web_interface:"
    pgrep -af web_interface
else
    echo "✅ Web interface fermato"
fi
echo ""

# 4. Ferma driver ROS2 (se attivo)
echo "[4/5] Fermo driver ROS2..."
if pgrep -f ur_ros2_control_node > /dev/null; then
    pkill -f ur_ros2_control_node
    echo "✅ Driver ROS2 fermato"
else
    echo "ℹ️  Driver ROS2 non attivo"
fi
echo ""

# 5. Libera porta 8080
echo "[5/5] Libero porta 8080..."
PORT=${WEB_PORT:-8080}
if lsof -i :$PORT > /dev/null 2>&1; then
    fuser -k $PORT/tcp 2>/dev/null || lsof -ti :$PORT | xargs kill -9 2>/dev/null
    sleep 1
    echo "✅ Porta $PORT liberata"
else
    echo "✅ Porta $PORT già libera"
fi
echo ""

# 6. Verifica finale
echo "[6/6] Verifica finale..."
WATCHDOG_RUNNING=$(pgrep -f watchdog_web_interface | wc -l)
WEB_RUNNING=$(pgrep -f web_interface | wc -l)
PORT_OCCUPIED=$(lsof -i :$PORT 2>/dev/null | wc -l)

if [ "$WATCHDOG_RUNNING" -eq 0 ] && [ "$WEB_RUNNING" -eq 0 ] && [ "$PORT_OCCUPIED" -eq 0 ]; then
    echo "=========================================="
    echo "✅ TUTTO FERMATO CORRETTAMENTE"
    echo "=========================================="
    echo ""
    echo "   Watchdog: Fermato"
    echo "   Web Interface: Fermato"
    echo "   Porta $PORT: Libera"
    echo ""
else
    echo "=========================================="
    echo "⚠️  ALCUNI PROCESSI ANCORA ATTIVI"
    echo "=========================================="
    echo ""
    if [ "$WATCHDOG_RUNNING" -gt 0 ]; then
        echo "   ❌ Watchdog ancora attivo:"
        pgrep -af watchdog_web_interface
    fi
    if [ "$WEB_RUNNING" -gt 0 ]; then
        echo "   ❌ Web interface ancora attivo:"
        pgrep -af web_interface
    fi
    if [ "$PORT_OCCUPIED" -gt 0 ]; then
        echo "   ❌ Porta $PORT ancora occupata:"
        lsof -i :$PORT
    fi
    echo ""
    echo "Prova manualmente:"
    echo "   pkill -9 -f watchdog_web_interface"
    echo "   pkill -9 -f web_interface"
    echo "   fuser -k $PORT/tcp"
fi

echo ""
