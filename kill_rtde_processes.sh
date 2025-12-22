#!/bin/bash
# Script per killare tutti i processi RTDE e liberare risorse prima di avviare driver ROS2

echo "=========================================="
echo "PULIZIA PROCESSI RTDE E ROS2"
echo "=========================================="

# 1. Kill processi ur_ros2_control_node
echo "[1/6] Kill processi ur_ros2_control_node..."
pkill -9 -f ur_ros2_control_node 2>/dev/null
sleep 1
echo "✅ Completato"

# 2. Kill processi ros2 launch ur_robot_driver
echo "[2/6] Kill processi ros2 launch..."
pkill -9 -f "ros2.*launch.*ur_robot_driver" 2>/dev/null
sleep 1
echo "✅ Completato"

# 3. Kill processi spawner controller
echo "[3/6] Kill processi spawner..."
pkill -9 -f "spawner.*controller" 2>/dev/null
sleep 1
echo "✅ Completato"

# 4. Kill processi controller_manager
echo "[4/6] Kill processi controller_manager..."
pkill -9 -f controller_manager 2>/dev/null
sleep 1
echo "✅ Completato"

# 5. Libera porta 50002
echo "[5/6] Libero porta 50002..."
fuser -k 50002/tcp 2>/dev/null || lsof -ti :50002 | xargs kill -9 2>/dev/null || true
sleep 1
echo "✅ Completato"

# 6. Verifica che tutto sia pulito
echo "[6/6] Verifica pulizia..."
REMAINING=$(pgrep -f "ur_ros2_control_node|ros2.*launch.*ur_robot_driver|spawner.*controller|controller_manager" | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "✅ Tutto pulito"
else
    echo "⚠️  Ancora $REMAINING processi attivi"
    pgrep -af "ur_ros2_control_node|ros2.*launch.*ur_robot_driver|spawner.*controller|controller_manager"
fi

echo "=========================================="
echo "PULIZIA COMPLETATA"
echo "=========================================="
