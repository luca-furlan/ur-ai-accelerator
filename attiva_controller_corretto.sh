#!/bin/bash
# Script per attivare forward_velocity_controller - FORMATO CORRETTO

echo "=================================================================================="
echo "🔧 ATTIVAZIONE FORWARD_VELOCITY_CONTROLLER"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Stato attuale controller..."
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>&1 | grep -A 1 "forward_velocity_controller" | grep state

echo ""
echo "2. Attivazione forward_velocity_controller..."
echo "   (Deattivando scaled_joint_trajectory_controller se necessario)"

# Formato corretto: deactivate scaled_joint_trajectory_controller e activate forward_velocity_controller
ros2 service call /controller_manager/switch_controllers \
    controller_manager_msgs/srv/SwitchControllers \
    "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}" 2>&1

sleep 2

echo ""
echo "3. Verifica stato dopo attivazione..."
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>&1 | grep -A 1 "forward_velocity_controller" | grep state

echo ""
echo "=================================================================================="
echo "✅ CONTROLLER ATTIVATO!"
echo "=================================================================================="
echo ""
echo "Ora prova il movimento:"
echo "  timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \\"
echo "      std_msgs/msg/Float64MultiArray \\"
echo "      '{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}'"
echo ""
echo "IL ROBOT DOVREBBE MUOVERSI!"
echo ""

