#!/bin/bash
# Passo 4: Test controllo ROS2

echo "================================================================================"
echo "PASSO 4: TEST CONTROLLO ROS2"
echo "================================================================================"
echo

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "1. Verifica topic ROS2 disponibili..."
echo "   Topic disponibili:"
ros2 topic list 2>&1 | grep -E "forward_velocity|scaled_joint_trajectory|servo_node|joint_states" | head -10
echo

echo "2. Verifica controller attivi..."
if command -v ros2controlcli &> /dev/null || ros2 pkg list | grep -q ros2controlcli; then
    echo "   Controller attivi:"
    ros2 control list_controllers 2>&1 | head -10
else
    echo "   ⚠️  ros2controlcli non installato"
    echo "   💡 Installa con: sudo apt install ros-humble-ros2controlcli"
fi
echo

echo "3. Test pubblicazione comando..."
echo "   Pubblico comando su /forward_velocity_controller/commands"
echo "   (Piccolo movimento joint 1: 0.05 rad/s)"
echo
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}"
echo
echo "   💡 Il robot dovrebbe muoversi leggermente (joint 1)"
echo "   Se si muove → ROS2 funziona!"
echo

echo "4. Monitor topic in tempo reale..."
echo "   (Premi CTRL+C per uscire)"
echo "   Muovi joystick nella web interface e vedrai messaggi qui sotto:"
echo
ros2 topic echo /forward_velocity_controller/commands










