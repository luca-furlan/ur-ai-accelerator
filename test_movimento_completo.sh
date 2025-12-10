#!/bin/bash
# Test completo movimento robot con verifica automatica

echo "=================================================================================="
echo "🧪 TEST COMPLETO MOVIMENTO ROBOT"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. Verifica controller
echo "1. Verifica controller attivo..."
CONTROLLER_STATE=$(ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "forward_velocity_controller" | grep "state=" | head -1)

if echo "$CONTROLLER_STATE" | grep -q "active"; then
    echo "   ✅ Controller ATTIVO"
else
    echo "   ❌ Controller NON attivo!"
    echo "   💡 Esegui: ./attiva_controller.sh"
    exit 1
fi

# 2. Leggi posizione iniziale
echo ""
echo "2. Leggo posizione INIZIALE..."
INITIAL_MSG=$(timeout 2 ros2 topic echo /joint_states --once 2>&1)
INITIAL_PAN=$(echo "$INITIAL_MSG" | grep -A 10 "position:" | grep -E "^-" | tail -1 | awk '{print $2}')
echo "   Posizione iniziale shoulder_pan_joint: $INITIAL_PAN"

# 3. Invia comando movimento
echo ""
echo "3. Invio comando movimento (joint 0: 0.15 rad/s per 4 secondi)..."
echo "   💡 Il robot dovrebbe ruotare la base!"
timeout 4 ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.15, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

# 4. Ferma movimento
echo ""
echo "4. Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

sleep 1

# 5. Leggi posizione finale
echo ""
echo "5. Leggo posizione FINALE..."
FINAL_MSG=$(timeout 2 ros2 topic echo /joint_states --once 2>&1)
FINAL_PAN=$(echo "$FINAL_MSG" | grep -A 10 "position:" | grep -E "^-" | tail -1 | awk '{print $2}')
echo "   Posizione finale shoulder_pan_joint: $FINAL_PAN"

# 6. Confronta posizioni
echo ""
echo "=================================================================================="
echo "📊 RISULTATO"
echo "=================================================================================="
echo ""

if [ -n "$INITIAL_PAN" ] && [ -n "$FINAL_PAN" ]; then
    DIFF=$(echo "$FINAL_PAN - $INITIAL_PAN" | bc -l 2>/dev/null || echo "0")
    ABS_DIFF=$(echo "$DIFF" | awk '{if ($1 < 0) print -$1; else print $1}')
    
    echo "   Posizione iniziale: $INITIAL_PAN rad"
    echo "   Posizione finale:   $FINAL_PAN rad"
    echo "   Differenza:         $DIFF rad"
    echo ""
    
    # Se la differenza è > 0.01 rad, il robot si è mosso
    if (( $(echo "$ABS_DIFF > 0.01" | bc -l 2>/dev/null || echo 0) )); then
        echo "   ✅ ROBOT SI È MOSSO! (differenza > 0.01 rad)"
        echo "   ✅ Il controller funziona correttamente!"
        exit 0
    else
        echo "   ❌ ROBOT NON SI È MOSSO (differenza < 0.01 rad)"
        echo "   💡 Verifica:"
        echo "      - Robot in RUNNING e PLAYING?"
        echo "      - External Control attivo?"
        echo "      - Controller attivo?"
        exit 1
    fi
else
    echo "   ⚠️  Impossibile leggere posizioni joint"
    echo "   💡 Verifica che il topic /joint_states sia disponibile"
    exit 1
fi

