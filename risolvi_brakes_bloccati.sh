#!/bin/bash
# Script per risolvere problemi di brakes bloccati e movimento UR robot

echo "=================================================================================="
echo "🔧 RISOLUZIONE: Brakes Bloccati e Movimento Robot"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica stato robot..."
python3 verifica_stato_dopo_ethernet_ip.py 2>&1 | grep -E "Robot in|Programma in|Remote Control" | head -5

echo ""
echo "2. Verifica brakes (sul Teach Pendant):"
echo "   💡 All'accensione dovresti sentire un 'click' quando i brakes si rilasciano"
echo "   💡 Se non senti il click, i brakes potrebbero essere ancora attivi"
echo ""

echo "3. Soluzione 1: Riavvia robot (rilascia brakes)"
echo "   → Vai sul Teach Pendant"
echo "   → Shutdown robot"
echo "   → Riavvia robot"
echo "   → Ascolta il 'click' dei brakes che si rilasciano"
echo ""

echo "4. Soluzione 2: Verifica modalità sicurezza..."
echo "   → Robot deve essere in RUNNING mode"
echo "   → Safety Mode deve essere NORMAL (non PROTECTIVE_STOP)"
echo ""

echo "5. Soluzione 3: Test movimento con comando più forte..."
echo "   💡 Prova con velocità molto alta per vedere se riesce a superare i brakes"
echo "   Premi INVIO per continuare o CTRL+C per annullare..."
read

echo ""
echo "   Test con velocità 2.0 rad/s (molto alta)..."
timeout 2 ros2 topic pub -r 50 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

echo ""
echo "   Fermo movimento..."
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' > /dev/null 2>&1

echo ""
echo "=================================================================================="
echo "📋 CHECKLIST COMPLETA"
echo "=================================================================================="
echo ""
echo "Verifica sul Teach Pendant:"
echo "  [ ] Robot in RUNNING mode (non POWER_OFF)"
echo "  [ ] Safety Mode = NORMAL (non PROTECTIVE_STOP)"
echo "  [ ] Programma in PLAYING"
echo "  [ ] Speed Scaling = 100%"
echo "  [ ] Nessun errore o warning visibile"
echo "  [ ] Brakes rilasciati (sentito 'click' all'accensione)"
echo ""
echo "Verifica fisica:"
echo "  [ ] Nessun ostacolo fisico che blocca i giunti"
echo "  [ ] Giunti si muovono liberamente quando spinti manualmente (con brakes rilasciati)"
echo "  [ ] Nessun cavo o detrito che impedisce movimento"
echo ""
echo "=================================================================================="
echo ""
echo "💡 SE IL ROBOT ANCORA NON SI MUOVE:"
echo ""
echo "1. Riavvia completamente il robot:"
echo "   → Shutdown dal Teach Pendant"
echo "   → Attendi 10 secondi"
echo "   → Riavvia"
echo "   → Ascolta il click dei brakes"
echo ""
echo "2. Prova movimento manuale sul Teach Pendant:"
echo "   → Se non riesci a muoverlo manualmente, è un problema hardware"
echo "   → Se riesci manualmente ma non via ROS2, è un problema software"
echo ""
echo "3. Verifica log driver ROS2:"
echo "   tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -i error"
echo ""

