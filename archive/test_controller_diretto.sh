#!/bin/bash
# Test diretto del controller ROS2

echo "=================================================================================="
echo "🧪 TEST DIRETTO CONTROLLER ROS2"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica topic esiste..."
ros2 topic list | grep forward_velocity
echo ""

echo "2. Verifica formato messaggio..."
ros2 topic type /forward_velocity_controller/commands
echo ""

echo "3. Test pubblicazione comando (joint 0: 0.05 rad/s per 2 secondi)..."
echo "   ⚠️  ATTENZIONE: Il robot dovrebbe muoversi!"
echo "   Premi CTRL+C per fermare prima se necessario"
echo ""
read -p "Premi INVIO per continuare o CTRL+C per annullare..."

# Pubblica comando continuo per 2 secondi
timeout 2 ros2 topic pub -r 125 /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}" 2>&1

echo ""
echo "4. Fermo movimento (velocità zero)..."
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
echo ""

echo "✅ Test completato"
echo ""
echo "💡 Se il robot NON si è mosso:"
echo "   - Verifica che il controller sia attivo"
echo "   - Verifica che il robot sia connesso al driver ROS2"
echo "   - Verifica che il programma External Control sia in PLAYING"







