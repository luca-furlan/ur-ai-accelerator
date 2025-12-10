#!/bin/bash
# Script semplice per attivare forward_velocity_controller

echo "=================================================================================="
echo "🔧 ATTIVAZIONE FORWARD_VELOCITY_CONTROLLER"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Attivazione controller..."
ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}" 2>&1 | grep -E "ok=True|ok=False"

sleep 1

echo ""
echo "Verifica stato..."
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "forward_velocity_controller" | grep state | grep -q "active" && \
    echo "✅ Controller ATTIVO!" || echo "❌ Controller NON attivo"

echo ""
echo "=================================================================================="
echo "✅ FATTO! Ora prova il joystick nella web interface!"
echo "=================================================================================="
echo ""

