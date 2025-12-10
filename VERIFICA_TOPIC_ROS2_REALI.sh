#!/bin/bash
# Script per verificare i topic ROS2 REALI disponibili nel driver UR

echo "================================================================================"
echo "VERIFICA TOPIC ROS2 REALI - Driver UR ROS2"
echo "================================================================================"
echo

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "1. Verifica se driver è installato..."
if ros2 pkg list | grep -q ur_robot_driver; then
    echo "   ✅ ur_robot_driver installato"
    DRIVER_PATH=$(ros2 pkg prefix ur_robot_driver)
    echo "   Path: $DRIVER_PATH"
    
    echo
    echo "2. Cerca file launch per vedere controller configurati..."
    if [ -d "$DRIVER_PATH/share/ur_robot_driver/launch" ]; then
        echo "   File launch trovati:"
        ls -la "$DRIVER_PATH/share/ur_robot_driver/launch/" | grep -E "ur_control|controllers"
        
        echo
        echo "3. Leggi configurazione controller..."
        if [ -f "$DRIVER_PATH/share/ur_robot_driver/config/ur_controllers.yaml" ]; then
            echo "   Contenuto ur_controllers.yaml:"
            cat "$DRIVER_PATH/share/ur_robot_driver/config/ur_controllers.yaml" | head -50
        fi
    fi
    
    echo
    echo "4. Cerca documentazione sui topic..."
    if [ -d "$DRIVER_PATH/share/ur_robot_driver/doc" ]; then
        echo "   File documentazione trovati:"
        find "$DRIVER_PATH/share/ur_robot_driver/doc" -name "*.rst" -o -name "*.md" | head -5
    fi
else
    echo "   ❌ ur_robot_driver NON installato"
    echo "   💡 Installa prima il driver"
fi

echo
echo "================================================================================"
echo "NOTA IMPORTANTE"
echo "================================================================================"
echo
echo "Per vedere i topic REALI disponibili quando il driver è in esecuzione:"
echo "  1. Avvia driver: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194"
echo "  2. In altro terminale: ros2 topic list"
echo "  3. Verifica controller: ros2 control list_controllers"
echo




