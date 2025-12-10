#!/bin/bash
# Test comandi semplici per UR robot ROS2

echo "=================================================================================="
echo "🧪 TEST COMANDI SEMPLICI UR ROBOT ROS2"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica speed scaling factor..."
echo "   (Deve essere > 0 per permettere movimento)"
ros2 topic echo /speed_scaling_status --once 2>&1 | grep -A 2 "speed_scaling_factor" || \
ros2 service call /io_and_status_controller/set_speed_slider \
    ur_msgs/srv/SetSpeedSliderFraction \
    "{speed_slider_fraction: 1.0}" 2>&1 | head -5

echo ""
echo "2. Imposta speed scaling a 100%..."
ros2 service call /io_and_status_controller/set_speed_slider \
    ur_msgs/srv/SetSpeedSliderFraction \
    "{speed_slider_fraction: 1.0}" 2>&1 | grep -E "success|ok" || echo "   Servizio non disponibile"

echo ""
echo "3. Test movimento con velocità più alta (0.3 rad/s)..."
echo "   💡 Se senti rumore ma non movimento, potrebbe essere speed scaling troppo basso"
timeout 3 ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}' 2>&1 | head -3

echo ""
echo "4. Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

echo ""
echo "=================================================================================="
echo "📋 PROSSIMI PASSI"
echo "=================================================================================="
echo ""
echo "Se il robot fa rumore ma non si muove:"
echo "  1. Verifica speed scaling sul Teach Pendant (deve essere > 0%)"
echo "  2. Verifica che non ci siano brakes attivi"
echo "  3. Verifica modalità sicurezza del robot"
echo "  4. Prova con velocità più alta (0.5 rad/s)"
echo ""

