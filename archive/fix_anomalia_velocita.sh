#!/bin/bash
# Fix anomalia velocità robot UR

echo "=================================================================================="
echo "🔧 FIX ANOMALIA VELOCITÀ ROBOT"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica stato robot dopo anomalia..."
echo "   💡 Se il robot è in anomalia, risolvi prima sul Teach Pendant"
echo ""

echo "2. Test con velocità MOLTO BASSA (0.05 rad/s)..."
echo "   💡 Velocità molto bassa per evitare anomalie"
echo ""

# Test con velocità molto bassa
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}' 2>&1 | head -5

echo ""
echo "3. Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

echo ""
echo "=================================================================================="
echo "📋 SOLUZIONI"
echo "=================================================================================="
echo ""
echo "1. ✅ RISOLVI ANOMALIA sul Teach Pendant:"
echo "   → Reset anomalia"
echo "   → Riavvia programma"
echo ""
echo "2. ✅ USA VELOCITÀ BASSA:"
echo "   → Massimo 0.1 rad/s per iniziare"
echo "   → Aumenta gradualmente se funziona"
echo ""
echo "3. ✅ VERIFICA LIMITI VELOCITÀ nel controller:"
echo "   → Il controller potrebbe avere limiti di sicurezza"
echo ""
echo "=================================================================================="
echo ""
echo "💡 PROSSIMI PASSI:"
echo "   1. Risolvi anomalia sul Teach Pendant"
echo "   2. Prova con velocità 0.05 rad/s"
echo "   3. Se funziona, aumenta gradualmente"
echo ""







