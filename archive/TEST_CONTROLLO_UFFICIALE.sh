#!/bin/bash
# Test controllo robot secondo documentazione ufficiale

set -e

echo "================================================================================"
echo "TEST CONTROLLO ROBOT - DOCUMENTAZIONE UFFICIALE"
echo "================================================================================"
echo

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Verifica driver in esecuzione
echo "1. VERIFICA DRIVER IN ESECUZIONE"
echo "--------------------------------"
if pgrep -f "ur_control.launch.py" > /dev/null; then
    echo "✅ Driver ROS2 in esecuzione"
else
    echo "❌ Driver ROS2 NON in esecuzione"
    echo "   Avvia con: ./AVVIA_DRIVER_UFFICIALE.sh"
    exit 1
fi
echo

# Verifica controller
echo "2. VERIFICA CONTROLLER"
echo "-----------------------"
if command -v ros2controlcli >/dev/null 2>&1 || ros2 control list_controllers >/dev/null 2>&1; then
    echo "Controller attivi:"
    ros2 control list_controllers 2>/dev/null || echo "   (comando non disponibile)"
else
    echo "⚠️  ros2controlcli non installato"
    echo "   Installa con: sudo apt install ros-humble-ros2controlcli"
fi
echo

# Verifica topic
echo "3. VERIFICA TOPIC ROS2"
echo "----------------------"
echo "Topic disponibili (attendi 2 secondi...):"
timeout 2 ros2 topic list 2>/dev/null | grep -E "(joint|trajectory|controller)" | head -10 || echo "   (nessun topic trovato)"
echo

# Verifica joint states
echo "4. VERIFICA JOINT STATES"
echo "------------------------"
echo "Lettura joint_states (attendi 1 secondo...):"
timeout 1 ros2 topic echo /joint_states --once 2>/dev/null | head -20 || echo "   (nessun dato disponibile)"
echo

echo "================================================================================"
echo "TEST COMPLETATO"
echo "================================================================================"
echo
echo "Per muovere il robot, usa uno di questi metodi:"
echo
echo "1. Test automatico (se disponibile):"
echo "   ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"
echo
echo "2. Pubblica comando manuale:"
echo "   ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory \\"
echo "       trajectory_msgs/msg/JointTrajectory \\"
echo "       '{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.0, -1.57, 1.57, -1.57, 1.57, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}'"
echo
echo "3. Usa la web interface (se disponibile):"
echo "   python3 remote_ur_control/web_interface.py"
echo









