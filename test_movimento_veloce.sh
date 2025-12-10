#!/bin/bash
# Test movimento robot con velocità alta

echo "=================================================================================="
echo "🧪 TEST MOVIMENTO ROBOT - VELOCITÀ ALTA"
echo "=================================================================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica controller attivo..."
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "forward_velocity_controller" | grep state | grep -q "active" && \
    echo "   ✅ Controller ATTIVO" || echo "   ❌ Controller NON attivo"

echo ""
echo "2. Test movimento con velocità 1.0 rad/s per 3 secondi..."
echo "   💡 GUARDA IL ROBOT - Dovrebbe ruotare la base!"
echo ""

timeout 3 ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

echo ""
echo "3. Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

echo ""
echo "=================================================================================="
echo "✅ Test completato"
echo "=================================================================================="
echo ""
echo "HAI VISTO IL ROBOT MUOVERSI? (s/n)"
read -p "> " risposta
if [ "$risposta" = "s" ] || [ "$risposta" = "S" ]; then
    echo ""
    echo "✅ PERFETTO! Il robot funziona!"
    echo "   Ora puoi usare il joystick nella web interface!"
else
    echo ""
    echo "❌ Il robot non si muove ancora"
    echo "   Verifica sul Teach Pendant:"
    echo "   - Speed Scaling deve essere almeno 50% (meglio 100%)"
    echo "   - Robot in RUNNING mode"
    echo "   - Programma in PLAYING"
fi
echo ""

