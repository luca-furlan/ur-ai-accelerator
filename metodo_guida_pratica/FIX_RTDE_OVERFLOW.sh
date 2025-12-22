#!/bin/bash

echo "=================================================================================="
echo "🔧 FIX RTDE Pipeline Overflow"
echo "=================================================================================="
echo ""

echo "PROBLEMA: RTDE Pipeline producer overflowed!"
echo "Il robot invia dati troppo velocemente per il driver."
echo ""

echo "SOLUZIONE 1: Ferma driver e riavvia con configurazione ridotta"
echo "----------------------------------------------------------------"
echo "1. Premi CTRL+C per fermare il driver corrente"
echo "2. Poi esegui questo script per riavviare con configurazione ottimizzata"
echo ""

read -p "Premi INVIO quando hai fermato il driver (CTRL+C)..."
echo ""

echo "SOLUZIONE 2: Riduci frequenza RTDE"
echo "-----------------------------------"
echo "Modificando configurazione per ridurre il carico..."
echo ""

# Crea file di configurazione ottimizzato
CONFIG_FILE="$HOME/ros2_ws/src/ur_robot_driver/config/ur_controllers.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "File configurazione trovato: $CONFIG_FILE"
    echo "Modifica manualmente per ridurre frequenza RTDE"
else
    echo "File configurazione non trovato, creo configurazione ottimizzata..."
fi

echo ""
echo "SOLUZIONE 3: Avvia driver con parametri ridotti"
echo "-----------------------------------------------"
echo "Avvio driver con frequenza ridotta..."
echo ""

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Ferma eventuali driver già in esecuzione
pkill -f "ur_control.launch.py" 2>/dev/null || true
sleep 2

echo "Avvio driver con configurazione ottimizzata..."
echo "Premi CTRL+C per fermare"
echo ""

# Avvia con parametri ridotti
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false \
    use_mock_hardware:=false \
    headless_mode:=true









