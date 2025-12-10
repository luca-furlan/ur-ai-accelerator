#!/bin/bash
# Script per verificare lo stato dell'AI Accelerator e testare il controllo robot

AI_ACCELERATOR_IP="192.168.10.191"
AI_ACCELERATOR_USER="lab"

echo "=========================================="
echo "VERIFICA STATO AI ACCELERATOR"
echo "=========================================="
echo "IP: $AI_ACCELERATOR_IP"
echo ""

ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP << 'ENDSSH'
echo "1. VERIFICA ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "   ✅ ROS2 Humble installato"
    source /opt/ros/humble/setup.bash 2>/dev/null
    ros2 --version 2>/dev/null || echo "   ⚠️  ROS2 non nel PATH"
else
    echo "   ❌ ROS2 Humble NON installato"
fi

echo ""
echo "2. VERIFICA WORKSPACE ROS2..."
if [ -d ~/ros2_ws ]; then
    echo "   ✅ Workspace ROS2 presente"
    if [ -d ~/ros2_ws/src/Universal_Robots_ROS2_Driver ]; then
        echo "   ✅ Universal Robots Driver presente"
    else
        echo "   ❌ Universal Robots Driver NON presente"
    fi
    if [ -d ~/ros2_ws/src/OrbbecSDK_ROS2 ]; then
        echo "   ✅ OrbbecSDK_ROS2 presente"
    else
        echo "   ⚠️  OrbbecSDK_ROS2 NON presente (opzionale)"
    fi
else
    echo "   ❌ Workspace ROS2 NON presente"
fi

echo ""
echo "3. VERIFICA PROGETTO MEKO..."
if [ -d ~/MekoAiAccelerator ]; then
    echo "   ✅ Directory progetto presente"
    if [ -d ~/MekoAiAccelerator/remote_ur_control ]; then
        echo "   ✅ remote_ur_control presente"
        if [ -f ~/MekoAiAccelerator/remote_ur_control/web_interface.py ]; then
            echo "   ✅ web_interface.py presente"
        fi
    fi
    if [ -f ~/MekoAiAccelerator/ros2_bridge_fixed.py ]; then
        echo "   ✅ ros2_bridge_fixed.py presente"
    fi
else
    echo "   ❌ Directory progetto NON presente"
fi

echo ""
echo "4. VERIFICA DIPENDENZE PYTHON..."
if python3 -c "import flask" 2>/dev/null; then
    echo "   ✅ Flask installato"
else
    echo "   ❌ Flask NON installato"
fi

if python3 -c "import rclpy" 2>/dev/null; then
    echo "   ✅ rclpy installato"
else
    echo "   ⚠️  rclpy NON installato (necessario per ROS2 bridge)"
fi

echo ""
echo "5. VERIFICA CONNESSIONE ROBOT..."
if ping -c 1 192.168.10.194 > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile (192.168.10.194)"
else
    echo "   ❌ Robot NON raggiungibile"
fi

echo ""
echo "6. VERIFICA PROCESSI IN ESECUZIONE..."
if pgrep -f "web_interface" > /dev/null; then
    echo "   ✅ Web interface in esecuzione"
else
    echo "   ⚠️  Web interface NON in esecuzione"
fi

if pgrep -f "ur_robot_driver" > /dev/null; then
    echo "   ✅ UR Robot Driver in esecuzione"
else
    echo "   ⚠️  UR Robot Driver NON in esecuzione"
fi

echo ""
echo "7. VERIFICA PORTE..."
if netstat -tuln 2>/dev/null | grep -q ":8080"; then
    echo "   ✅ Porta 8080 (web interface) in uso"
else
    echo "   ⚠️  Porta 8080 libera"
fi

echo ""
echo "=========================================="
echo "RIEPILOGO"
echo "=========================================="
ENDSSH

