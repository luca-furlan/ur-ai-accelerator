#!/bin/bash
# Fix completo per ROS2 e ur_rtde

echo "================================================================================"
echo "FIX ROS2 E UR_RTDE - COMPLETO"
echo "================================================================================"
echo

# 1. Installa ur_rtde
echo "1. INSTALLAZIONE UR_RTDE..."
if python3 -c "import ur_rtde" 2>/dev/null; then
    echo "   ✅ ur_rtde già installato"
    python3 -c "import ur_rtde; print(f'   Versione: {ur_rtde.__version__}')" 2>/dev/null || echo "   (versione non disponibile)"
else
    echo "   ⚠️ ur_rtde non installato, installo..."
    pip install ur_rtde
    if python3 -c "import ur_rtde" 2>/dev/null; then
        echo "   ✅ ur_rtde installato con successo"
    else
        echo "   ❌ Errore installazione ur_rtde"
        exit 1
    fi
fi
echo

# 2. Verifica ROS2
echo "2. VERIFICA ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "   ✅ ROS2 Humble installato"
    source /opt/ros/humble/setup.bash
    echo "   ROS_DISTRO: $ROS_DISTRO"
else
    echo "   ❌ ROS2 non trovato in /opt/ros/humble/"
    exit 1
fi
echo

# 3. Verifica rclpy
echo "3. VERIFICA RCLPY..."
if python3 -c "import rclpy" 2>/dev/null; then
    echo "   ✅ rclpy disponibile"
else
    echo "   ❌ rclpy non disponibile"
    echo "   💡 Installa: sudo apt install ros-humble-rclpy"
    exit 1
fi
echo

# 4. Verifica workspace ROS2
echo "4. VERIFICA WORKSPACE ROS2..."
if [ -d ~/ros2_ws ]; then
    echo "   ✅ Workspace ROS2 trovato: ~/ros2_ws"
    
    # Verifica se compilato
    if [ -d ~/ros2_ws/install ]; then
        echo "   ✅ Workspace compilato"
    else
        echo "   ⚠️ Workspace non compilato"
        echo "   💡 Compila con: cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build"
    fi
else
    echo "   ⚠️ Workspace ROS2 non trovato"
fi
echo

# 5. Verifica ros2_bridge_fixed.py
echo "5. VERIFICA ROS2 BRIDGE..."
if [ -f ~/MekoAiAccelerator/ros2_bridge_fixed.py ]; then
    echo "   ✅ ros2_bridge_fixed.py trovato"
else
    echo "   ❌ ros2_bridge_fixed.py non trovato"
    exit 1
fi
echo

# 6. Kill processi esistenti
echo "6. PULIZIA PROCESSI ESISTENTI..."
pkill -f "ros2_bridge_fixed.py" 2>/dev/null && echo "   ✅ Processi ros2_bridge killati" || echo "   Nessun processo ros2_bridge attivo"
pkill -f "web_interface.py" 2>/dev/null && echo "   ✅ Processi web_interface killati" || echo "   Nessun processo web_interface attivo"
sleep 2
echo

# 7. Avvia ROS2 Bridge in background
echo "7. AVVIO ROS2 BRIDGE..."
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash

# Avvia in background
nohup python3 ros2_bridge_fixed.py > /tmp/ros2_bridge.log 2>&1 &
ROS2_BRIDGE_PID=$!
sleep 3

if ps -p $ROS2_BRIDGE_PID > /dev/null; then
    echo "   ✅ ROS2 Bridge avviato (PID: $ROS2_BRIDGE_PID)"
    echo "   Log: /tmp/ros2_bridge.log"
else
    echo "   ❌ ROS2 Bridge non si è avviato"
    echo "   💡 Controlla log: cat /tmp/ros2_bridge.log"
    exit 1
fi
echo

# 8. Verifica che ROS2 Bridge funzioni
echo "8. VERIFICA ROS2 BRIDGE..."
sleep 2
if ps -p $ROS2_BRIDGE_PID > /dev/null; then
    echo "   ✅ ROS2 Bridge ancora attivo"
    
    # Verifica log per errori
    if grep -i "error\|fail" /tmp/ros2_bridge.log 2>/dev/null | tail -3; then
        echo "   ⚠️ Trovati errori nel log"
    else
        echo "   ✅ Nessun errore nel log"
    fi
else
    echo "   ❌ ROS2 Bridge si è fermato"
    echo "   💡 Controlla log: cat /tmp/ros2_bridge.log"
fi
echo

# 9. Verifica topic ROS2
echo "9. VERIFICA TOPIC ROS2..."
source /opt/ros/humble/setup.bash
if ros2 topic list 2>/dev/null | grep -q "forward_velocity_controller\|speedj"; then
    echo "   ✅ Topic ROS2 disponibili"
    ros2 topic list 2>/dev/null | grep -E "forward_velocity_controller|speedj" | head -3
else
    echo "   ⚠️ Topic ROS2 non trovati (potrebbe essere normale se driver non è attivo)"
fi
echo

echo "================================================================================"
echo "✅ FIX COMPLETATO"
echo "================================================================================"
echo
echo "📍 PROSSIMI PASSI:"
echo "   1. Verifica Web Interface: http://192.168.10.191:8081"
echo "   2. ROS2 Bridge dovrebbe essere attivo"
echo "   3. Se ancora non funziona, avvia driver UR ROS2:"
echo "      cd ~/ros2_ws"
echo "      source install/setup.bash"
echo "      ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194"
echo





