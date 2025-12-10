#!/bin/bash

# Avvia driver ROS2 con frequenza RTDE ridotta per evitare overflow
# Utile quando EtherNet/IP è abilitato

set +e

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

cd ~/MekoAiAccelerator/metodo_guida_pratica || exit 1

# Ferma tutto
echo "Fermo processi esistenti..."
pkill -f "ur_robot_driver\|ur_control.launch" 2>/dev/null
sleep 3

# Crea configurazione con frequenza ridotta
mkdir -p /tmp/ur_config
cat > /tmp/ur_config/ur5e_low_freq.yaml << 'EOF'
controller_manager:
  ros__parameters:
    update_rate: 125  # Hz (ridotto da 500 per evitare overflow con EtherNet/IP)
EOF

echo "=================================================================================="
echo "🚀 AVVIO DRIVER ROS2 CON FREQUENZA RTDE RIDOTTA (125 Hz)"
echo "=================================================================================="
echo ""
echo "Questa configurazione può permettere a EtherNet/IP e ROS2 di coesistere"
echo ""
echo "Configurazione:"
echo "  - Frequenza RTDE: 125 Hz (ridotta da 500 Hz)"
echo "  - Robot IP: $ROBOT_IP"
echo "  - Robot Type: $ROBOT_TYPE"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

# Configura ROS2
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Avvia driver con configurazione personalizzata
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$ROBOT_TYPE \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false \
    --params-file /tmp/ur_config/ur5e_low_freq.yaml

