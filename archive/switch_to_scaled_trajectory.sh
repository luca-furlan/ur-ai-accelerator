#!/bin/bash
# Switch a scaled_joint_trajectory_controller per movimento fluido e sicuro

echo "=================================================================================="
echo "🔄 SWITCH A SCALED_JOINT_TRAJECTORY_CONTROLLER"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica controller attuali..."
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -E "forward_velocity_controller|scaled_joint_trajectory_controller" | \
    grep -E "state=" | head -2

echo ""
echo "2. Switch a scaled_joint_trajectory_controller..."
ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: ['forward_velocity_controller'], strictness: 1}" 2>&1 | grep -E "ok=True|ok=False" || echo "   Switch completato"

sleep 1

echo ""
echo "3. Verifica controller attivo..."
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "scaled_joint_trajectory_controller" | grep state | grep -q "active" && \
    echo "   ✅ scaled_joint_trajectory_controller ATTIVO" || echo "   ❌ Controller NON attivo"

echo ""
echo "=================================================================================="
echo "✅ FATTO!"
echo "=================================================================================="
echo ""
echo "Ora riavvia la web interface per usare il nuovo controller:"
echo "  pkill -f web_interface"
echo "  cd ~/MekoAiAccelerator"
echo "  ./avvia_web_interface_joystick.sh"
echo ""
echo "Il movimento sarà più fluido e sicuro!"
echo ""







