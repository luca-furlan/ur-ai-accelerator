#!/bin/bash
# Test movimento secondo guida pratica
# Fonte: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================================================================"
echo "TEST MOVIMENTO UR5e CON ROS2"
echo "Guida: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db"
echo "================================================================================"
echo

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Verifica driver in esecuzione
echo "1. VERIFICA DRIVER ROS2 IN ESECUZIONE"
echo "--------------------------------------"
if pgrep -f "ur_control.launch.py" > /dev/null; then
    echo "✅ Driver ROS2 in esecuzione"
else
    echo "❌ Driver ROS2 NON in esecuzione"
    echo "   Avvia con: ./SETUP_COMPLETO.sh"
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

# Test movimento secondo guida
echo "================================================================================"
echo "3. TEST MOVIMENTO (SECONDO GUIDA)"
echo "================================================================================"
echo
echo "Secondo la guida, usa questo comando per test movimento:"
echo
echo "ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"
echo
echo "Questo invierà un loop infinito di comandi di movimento al robot."
echo
read -p "Vuoi avviare il test movimento? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo
    echo "Avvio test movimento..."
    echo "Premi CTRL+C per fermare"
    echo
    ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
else
    echo
    echo "Test movimento annullato."
    echo
    echo "Per avviarlo manualmente:"
    echo "  ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"
fi










