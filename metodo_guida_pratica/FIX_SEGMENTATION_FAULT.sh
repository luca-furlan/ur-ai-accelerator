#!/bin/bash

# Fix per segmentation fault del driver ROS2
# Basato su Issue #31 (EtherNet/IP) e problemi RTDE

echo "=================================================================================="
echo "🔧 FIX SEGMENTATION FAULT DRIVER ROS2"
echo "=================================================================================="
echo ""

# 1. Ferma tutto
echo "1. Fermo tutti i processi ROS2..."
pkill -f "ur_robot_driver\|ur_control.launch\|ros2" 2>/dev/null
sleep 2
echo "   ✅ Processi fermati"
echo ""

# 2. Verifica processi RTDE
echo "2. Verifica processi RTDE attivi..."
RTDE_PROCESSES=$(ps aux | grep -E "rtde|ur_rtde" | grep -v grep || true)
if [ -z "$RTDE_PROCESSES" ]; then
    echo "   ✅ Nessun altro processo RTDE attivo"
else
    echo "   ⚠️  Altri processi RTDE trovati:"
    echo "$RTDE_PROCESSES"
    echo "   Chiudo processi RTDE..."
    pkill -f "rtde\|ur_rtde" 2>/dev/null
    sleep 1
fi
echo ""

# 3. Verifica porta 50002 occupata
echo "3. Verifica porta 50002..."
if netstat -tuln | grep -q ":50002 "; then
    echo "   ⚠️  Porta 50002 già in uso"
    if command -v lsof > /dev/null 2>&1; then
        echo "   Processi che usano porta 50002:"
        sudo lsof -i :50002 2>/dev/null || echo "   Impossibile verificare (serve sudo)"
    fi
else
    echo "   ✅ Porta 50002 libera"
fi
echo ""

# 4. Verifica EtherNet/IP (istruzioni)
echo "4. VERIFICA SUL TEACH PENDANT (IMPORTANTE!):"
echo "   - Vai su: Installation → Fieldbus"
echo "   - EtherNet/IP deve essere DISABILITATO ❌"
echo "   - PROFINET deve essere DISABILITATO ❌"
echo "   - Solo Ethernet normale può essere abilitato ✅"
echo ""
echo "   Questo è la causa più comune di segmentation fault (Issue #31)!"
echo ""

# 5. Verifica calibration
echo "5. Verifica calibration..."
if [ -f ~/my_robot_calibration.yaml ]; then
    echo "   ✅ File calibration trovato: ~/my_robot_calibration.yaml"
    echo "   Se il driver dice 'calibration parameters don't match',"
    echo "   rigenera calibration:"
    echo "   ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.10.194 target_filename:=\${HOME}/my_robot_calibration.yaml"
else
    echo "   ⚠️  File calibration non trovato"
    echo "   Potrebbe causare problemi di precisione"
fi
echo ""

# 6. Avvia driver con parametri sicuri
echo "6. Avvio driver ROS2 con parametri sicuri..."
cd ~/MekoAiAccelerator/metodo_guida_pratica

source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo ""
echo "Avvio driver (premi CTRL+C per fermare)..."
echo ""

# Avvia con headless mode se disponibile
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false \
    2>&1 | tee /tmp/driver_ros2.log

echo ""
echo "=================================================================================="
echo "📋 SE ANCORA VA IN CRASH"
echo "=================================================================================="
echo ""
echo "1. Verifica EtherNet/IP DISABILITATO sul robot (Issue #31)"
echo "2. Verifica Remote Control abilitato"
echo "3. Verifica nessun altro processo RTDE attivo"
echo "4. Rigenera calibration se necessario"
echo ""
echo "Log driver: tail -f /tmp/driver_ros2.log"
echo ""








