#!/bin/bash
# Script per fixare overflow RTDE pipeline

echo "================================================================================"
echo "FIX: Pipeline Producer Overflowed - RTDE Data Pipeline"
echo "================================================================================"
echo

echo "1. Fermo tutti i processi che usano RTDE..."
pkill -f ur_robot_driver
pkill -f web_interface
pkill -f rtde
pkill -f ur_rtde

sleep 2

echo "   ✅ Processi fermati"
echo

echo "2. Verifica processi rimanenti..."
RTDE_PROCESSES=$(ps aux | grep -E "rtde|ur_rtde|ur_robot_driver" | grep -v grep | wc -l)
if [ "$RTDE_PROCESSES" -gt 0 ]; then
    echo "   ⚠️  Ancora $RTDE_PROCESSES processi RTDE attivi:"
    ps aux | grep -E "rtde|ur_rtde|ur_robot_driver" | grep -v grep
    echo
    echo "   💡 Chiudi manualmente questi processi"
else
    echo "   ✅ Nessun processo RTDE attivo"
fi
echo

echo "================================================================================"
echo "SCELTA: Come vuoi controllare il robot?"
echo "================================================================================"
echo
echo "OPZIONE 1: ROS2 Driver (consigliato per controllo avanzato)"
echo "  - Avvia driver UR ROS2"
echo "  - Usa ROS2 topics per controllo"
echo "  - NON avviare web interface (usa RTDE)"
echo
echo "OPZIONE 2: Web Interface con Socket (più semplice)"
echo "  - Avvia web interface"
echo "  - Usa socket diretto (porta 30002)"
echo "  - NON avviare driver UR ROS2 (usa RTDE)"
echo
echo "================================================================================"
echo "IMPORTANTE"
echo "================================================================================"
echo
echo "⚠️  RTDE può essere usato da UN SOLO processo alla volta!"
echo
echo "Se avvii driver UR ROS2 → NON avviare web interface"
echo "Se avvii web interface → NON avviare driver UR ROS2"
echo
echo "================================================================================"
echo "PER AVVIARE ROS2 DRIVER:"
echo "================================================================================"
echo
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "    ur_type:=ur5e \\"
echo "    robot_ip:=192.168.10.194 \\"
echo "    launch_rviz:=false"
echo
echo "================================================================================"
echo "PER AVVIARE WEB INTERFACE (socket, senza RTDE):"
echo "================================================================================"
echo
echo "cd ~/MekoAiAccelerator"
echo "export UR_ROBOT_IP=192.168.10.194"
echo "export WEB_PORT=8081"
echo "python3 -m remote_ur_control.web_interface"
echo
echo "================================================================================"










