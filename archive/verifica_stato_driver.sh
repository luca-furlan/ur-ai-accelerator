#!/bin/bash

# Script per verificare lo stato del driver ROS2 e porta 50002

echo "=================================================================================="
echo "🔍 VERIFICA STATO DRIVER ROS2 E PORTA 50002"
echo "=================================================================================="
echo ""

# 1. Verifica driver ROS2
echo "1. Driver ROS2:"
if pgrep -f "ur_ros2_control_node" > /dev/null; then
    PID=$(pgrep -f "ur_ros2_control_node" | head -1)
    echo "   ✅ Driver ROS2 ATTIVO (PID: $PID)"
    
    # Verifica se il processo è ancora vivo
    if ps -p $PID > /dev/null 2>&1; then
        echo "   ✅ Processo vivo"
    else
        echo "   ❌ Processo morto (zombie?)"
    fi
else
    echo "   ❌ Driver ROS2 NON attivo"
fi

echo ""

# 2. Verifica porta 50002
echo "2. Porta 50002:"
if netstat -tuln 2>/dev/null | grep -q ":50002 "; then
    echo "   ✅ Porta 50002 APERTA"
    netstat -tuln | grep ":50002 " | head -1
else
    echo "   ❌ Porta 50002 CHIUSA"
fi

echo ""

# 3. Verifica log driver
echo "3. Ultimi errori nel log driver (/tmp/ros2_driver.log):"
if [ -f /tmp/ros2_driver.log ]; then
    echo "   Ultime 20 righe del log:"
    tail -20 /tmp/ros2_driver.log | sed 's/^/   /'
else
    echo "   ⚠️  File di log non trovato"
fi

echo ""

# 4. Verifica connessioni attive sulla porta 50002
echo "4. Connessioni attive sulla porta 50002:"
if netstat -tn 2>/dev/null | grep ":50002 " > /dev/null; then
    echo "   Connessioni trovate:"
    netstat -tn | grep ":50002 " | sed 's/^/   /'
else
    echo "   Nessuna connessione attiva"
fi

echo ""

# 5. Verifica ROS2 topics
echo "5. ROS2 Topics disponibili:"
if command -v ros2 > /dev/null 2>&1; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    if ros2 topic list 2>/dev/null | grep -q "joint_states"; then
        echo "   ✅ ROS2 topics disponibili"
        echo "   Topics trovati:"
        ros2 topic list 2>/dev/null | grep -E "(joint|controller)" | sed 's/^/     /' | head -5
    else
        echo "   ❌ ROS2 topics non disponibili (driver potrebbe essere crashato)"
    fi
else
    echo "   ⚠️  ROS2 non trovato nel PATH"
fi

echo ""
echo "=================================================================================="
echo "💡 SUGGERIMENTI:"
echo "=================================================================================="
echo ""

if ! pgrep -f "ur_ros2_control_node" > /dev/null; then
    echo "❌ Driver ROS2 non attivo!"
    echo "   Avvia con: ./avvia_driver_ros2.sh"
elif ! netstat -tuln 2>/dev/null | grep -q ":50002 "; then
    echo "❌ Porta 50002 non aperta!"
    echo "   Il driver potrebbe essere crashato. Controlla il log:"
    echo "   tail -50 /tmp/ros2_driver.log"
    echo "   Riavvia con: ./avvia_driver_ros2.sh"
else
    echo "✅ Driver ROS2 attivo e porta 50002 aperta"
    echo "   Se External Control non funziona, verifica:"
    echo "   1. IP Host sul Teach Pendant: 192.168.10.191"
    echo "   2. Porta sul Teach Pendant: 50002"
    echo "   3. Robot in Remote Control mode"
fi

echo ""







