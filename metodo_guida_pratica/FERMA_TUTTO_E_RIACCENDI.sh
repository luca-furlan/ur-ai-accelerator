#!/bin/bash

echo "=================================================================================="
echo "🛑 FERMA TUTTO E LIBERA PORTA 50002"
echo "=================================================================================="
echo ""

echo "1. Fermo tutti i processi sulla porta 50002..."
lsof -ti:50002 2>/dev/null | xargs kill -9 2>/dev/null || true
fuser -k 50002/tcp 2>/dev/null || true
pkill -f 'nc -l -p 50002' 2>/dev/null || true
echo "   ✅ Processi sulla porta 50002 fermati"

echo ""
echo "2. Fermo tutti i processi ROS2..."
pkill -f 'ur_robot_driver' 2>/dev/null || true
pkill -f 'ur_control.launch' 2>/dev/null || true
pkill -f 'controller_manager' 2>/dev/null || true
pkill -f 'spawner' 2>/dev/null || true
sleep 2
echo "   ✅ Processi ROS2 fermati"

echo ""
echo "3. Verifica porta 50002 libera..."
if netstat -tuln | grep -q ":50002"; then
    echo "   ⚠️  Porta 50002 ancora in uso!"
    echo "   Processi che usano la porta:"
    lsof -i:50002 2>/dev/null || netstat -tuln | grep ":50002"
else
    echo "   ✅ Porta 50002 libera!"
fi

echo ""
echo "=================================================================================="
echo "✅ FATTO! Ora puoi riavviare il driver con: ./START_RAPIDO.sh"
echo "=================================================================================="









