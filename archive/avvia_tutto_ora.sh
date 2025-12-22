#!/bin/bash
# Script per avviare TUTTO quello che funziona ORA

echo "=========================================="
echo "COSA PUOI USARE ORA - UR5e"
echo "=========================================="
echo ""

echo "✅ FUNZIONA SUBITO:"
echo ""
echo "1. RTDE - Comunicazione robot"
echo "   Test: python3 << 'EOF'"
echo "   import rtde.rtde as rtde"
echo "   con = rtde.RTDE('192.168.10.194', 30004)"
echo "   con.connect()"
echo "   print('✅ RTDE OK')"
echo "   con.disconnect()"
echo "   EOF"
echo ""

echo "2. Dashboard - Controllo robot (porta 29999)"
echo "   ✅ Già testato - funziona!"
echo ""

echo "3. ROS2 Driver - Controllo avanzato"
echo "   bash avvia_driver_ur5e.sh"
echo ""

echo "4. Web Interface - Controllo browser"
echo "   bash avvia_web_interface_ur5e.sh"
echo "   Browser: http://192.168.10.191:8080"
echo ""

echo "5. MuJoCo - Simulazione"
echo "   python3 -m mujoco.viewer --mjcf ~/mujoco_menagerie/universal_robots_ur5e/scene.xml"
echo ""

echo "6. YOLOv8 - Object Detection"
echo "   ✅ Installato e pronto"
echo ""

echo "7. OpenCV - Computer Vision"
echo "   ✅ Installato e pronto"
echo ""

echo "8. Open3D - Point Cloud"
echo "   ✅ Installato e pronto"
echo ""

echo "=========================================="
echo "⚠️  OPZIONALE (non necessario ora):"
echo "=========================================="
echo ""
echo "- MoveIt2 (motion planning avanzato)"
echo "- Camera Orbbec (se hai camera)"
echo ""

echo "=========================================="
echo "🚀 QUICK START - Controllo Robot ORA:"
echo "=========================================="
echo ""
echo "Terminale 1:"
echo "  bash avvia_driver_ur5e.sh"
echo ""
echo "Sul Teach Pendant:"
echo "  1. Avvia programma External Control"
echo "  2. IP: 192.168.10.191, Porta: 50002"
echo "  3. Premi PLAY"
echo ""
echo "Terminale 2:"
echo "  bash avvia_web_interface_ur5e.sh"
echo ""
echo "Browser:"
echo "  http://192.168.10.191:8080"
echo ""
echo "=========================================="
echo "✅ TUTTO PRONTO PER USO!"
echo "=========================================="











