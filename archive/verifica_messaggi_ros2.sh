#!/bin/bash
# Script per verificare se i messaggi ROS2 arrivano correttamente

echo "=================================================================================="
echo "🔍 VERIFICA MESSAGGI ROS2 IN TEMPO REALE"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica topic esiste..."
if ros2 topic list | grep -q "forward_velocity_controller/commands"; then
    echo "   ✅ Topic esiste"
else
    echo "   ❌ Topic NON esiste"
    exit 1
fi

echo ""
echo "2. Verifica publisher/subscriber..."
INFO=$(ros2 topic info /forward_velocity_controller/commands 2>&1)
echo "$INFO" | grep -E "Publisher|Subscription"
echo ""

echo "3. Monitor messaggi (premi CTRL+C per uscire)..."
echo "   💡 MUOVI IL JOYSTICK nella web interface e vedrai i valori qui sotto"
echo "   💡 Se vedi solo zeri anche quando muovi il joystick, i messaggi non arrivano"
echo ""
echo "   Valori attesi quando muovi joystick:"
echo "   - Dovresti vedere valori diversi da zero (es. 0.05, -0.1, etc.)"
echo "   - Se vedi solo 0.0, 0.0, 0.0... → problema nella pubblicazione"
echo ""

# Monitora per 10 secondi
timeout 10 ros2 topic echo /forward_velocity_controller/commands 2>&1 | grep -E "data:|^-" | head -30

echo ""
echo "=================================================================================="
echo "📋 RISULTATO"
echo "=================================================================================="
echo ""
echo "Se hai visto valori diversi da zero quando muovevi il joystick:"
echo "  ✅ Messaggi arrivano correttamente"
echo "  ❓ Il problema potrebbe essere nel controller o nella configurazione robot"
echo ""
echo "Se hai visto solo zeri anche quando muovevi il joystick:"
echo "  ❌ Messaggi NON arrivano al topic"
echo "  💡 Problema nel bridge ROS2 o nella pubblicazione"
echo ""







