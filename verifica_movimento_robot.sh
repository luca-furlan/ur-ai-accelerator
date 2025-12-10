#!/bin/bash
# Script per verificare se il robot si muove

echo "=================================================================================="
echo "🔍 VERIFICA MOVIMENTO ROBOT"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica controller attivo..."
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "forward_velocity_controller" | grep state | grep -q "active" && \
    echo "   ✅ Controller ATTIVO" || echo "   ❌ Controller NON attivo"

echo ""
echo "2. Leggi posizione joint INIZIALE..."
INITIAL=$(timeout 2 ros2 topic echo /joint_states --once 2>&1 | grep -A 6 "position:" | head -7)
echo "$INITIAL" | grep -E "position:|shoulder_pan"

echo ""
echo "3. Invio comando movimento (joint 0: 0.15 rad/s per 3 secondi)..."
echo "   💡 GUARDA IL ROBOT - Dovrebbe ruotare la base!"
timeout 3 ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.15, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1 &

sleep 3

echo ""
echo "4. Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

echo ""
echo "5. Leggi posizione joint FINALE..."
sleep 1
FINAL=$(timeout 2 ros2 topic echo /joint_states --once 2>&1 | grep -A 6 "position:" | head -7)
echo "$FINAL" | grep -E "position:|shoulder_pan"

echo ""
echo "=================================================================================="
echo "📋 RISULTATO"
echo "=================================================================================="
echo ""
echo "💡 Confronta le posizioni INITIAL e FINAL del joint shoulder_pan_joint"
echo "   Se sono diverse → Robot SI È MOSSO ✅"
echo "   Se sono uguali → Robot NON si è mosso ❌"
echo ""
echo "HAI VISTO IL ROBOT MUOVERSI FISICAMENTE? (s/n)"
read -p "> " risposta
if [ "$risposta" = "s" ] || [ "$risposta" = "S" ]; then
    echo "✅ PERFETTO! Il controller funziona!"
else
    echo "❌ Il robot non si muove. Verifica:"
    echo "   - Controller attivo?"
    echo "   - Robot in RUNNING e PLAYING?"
    echo "   - External Control attivo?"
fi
echo ""

