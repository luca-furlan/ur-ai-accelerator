#!/bin/bash
# Diagnostica robot che fa rumore ma non si muove

echo "=================================================================================="
echo "🔍 DIAGNOSTICA: Robot Fa Rumore Ma Non Si Muove"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica controller attivo..."
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "forward_velocity_controller" | grep state | grep -q "active" && \
    echo "   ✅ Controller ATTIVO" || echo "   ❌ Controller NON attivo"

echo ""
echo "2. Verifica posizione corrente joint..."
JOINT_STATE=$(timeout 2 ros2 topic echo /joint_states --once 2>&1 | grep -A 10 "position:" | head -8)
echo "$JOINT_STATE"
echo ""

echo "3. Test movimento JOINT DIVERSO (joint 1 invece di joint 0)..."
echo "   💡 Prova a muovere un altro giunto"
echo "   Premi INVIO per continuare o CTRL+C per annullare..."
read

echo "   Test movimento joint 1 (shoulder_lift) con velocità 0.5 rad/s..."
timeout 3 ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]}'

echo ""
echo "   Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

echo ""
echo "=================================================================================="
echo "📋 POSSIBILI CAUSE"
echo "=================================================================================="
echo ""
echo "Se il robot fa rumore ma non si muove:"
echo ""
echo "1. ⚠️  SPEED SCALING TROPPO BASSO sul Teach Pendant"
echo "   → Vai sul Teach Pendant"
echo "   → Imposta Speed Scaling a 100%"
echo "   → Questo è probabilmente il problema principale!"
echo ""
echo "2. ⚠️  BRAKES ATTIVI"
echo "   → Verifica sul Teach Pendant se ci sono brakes attivi"
echo "   → Rilascia manualmente se necessario"
echo ""
echo "3. ⚠️  POSIZIONE ROBOT"
echo "   → Il robot potrebbe essere in una posizione che impedisce movimento"
echo "   → Prova a muovere manualmente sul Teach Pendant"
echo ""
echo "4. ⚠️  LIMITI DI SICUREZZA"
echo "   → Verifica che non ci siano errori sicurezza sul Teach Pendant"
echo "   → Robot deve essere in RUNNING mode"
echo ""
echo "=================================================================================="
echo ""
echo "💡 AZIONE IMMEDIATA:"
echo "   Vai sul Teach Pendant e imposta Speed Scaling a 100%"
echo "   Poi riprova il movimento"
echo ""

