#!/bin/bash
# Script robusto per attivare forward_velocity_controller
# Usa switch_controller.py che verifica lo stato PRIMA di fare switch

echo "=================================================================================="
echo "🔧 ATTIVAZIONE FORWARD_VELOCITY_CONTROLLER"
echo "=================================================================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Verifica stato corrente..."
ros2 control list_controllers 2>&1 | grep -E "forward_velocity_controller|scaled_joint_trajectory_controller" || echo "   Nessun controller trovato"

echo ""
echo "Attivazione controller (usa switch_controller.py robusto)..."
python3 switch_controller.py forward_velocity_controller scaled_joint_trajectory_controller

EXIT_CODE=$?

sleep 1

echo ""
echo "Verifica stato finale..."
ros2 control list_controllers 2>&1 | grep -A 1 "forward_velocity_controller" | grep -q "active" && \
    echo "✅ Controller ATTIVO!" || echo "⚠️ Controller potrebbe non essere attivo"

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "=================================================================================="
    echo "✅ FATTO! Controller attivato correttamente!"
    echo "   Ora prova il joystick nella web interface: http://192.168.10.191:8080"
    echo "=================================================================================="
else
    echo "=================================================================================="
    echo "⚠️ ATTENZIONE: Potrebbero esserci problemi con l'attivazione"
    echo "   Verifica i log sopra per dettagli"
    echo "=================================================================================="
fi
echo ""








