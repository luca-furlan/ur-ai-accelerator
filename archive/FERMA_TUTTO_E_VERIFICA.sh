#!/bin/bash
# Script per fermare tutto e verificare cosa sta usando RTDE

echo "================================================================================"
echo "FERMA TUTTO E VERIFICA PROCESSI RTDE"
echo "================================================================================"
echo

echo "1. Fermo tutti i processi..."
pkill -9 -f ur_robot_driver
pkill -9 -f web_interface
pkill -9 -f rtde
pkill -9 -f ur_rtde
pkill -9 -f ros2

sleep 3

echo "   ✅ Processi fermati"
echo

echo "2. Verifica processi rimanenti..."
PROCESSES=$(ps aux | grep -E "rtde|ur_rtde|ur_robot_driver|web_interface|ros2" | grep -v grep)
if [ -z "$PROCESSES" ]; then
    echo "   ✅ Nessun processo RTDE/ROS2 attivo"
else
    echo "   ⚠️  Processi ancora attivi:"
    echo "$PROCESSES"
    echo
    echo "   💡 Chiudi manualmente questi processi"
fi
echo

echo "3. Verifica connessioni RTDE al robot..."
ROBOT_IP="192.168.10.194"
echo "   Verifica porta 30004 (RTDE)..."
if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/30004" 2>/dev/null; then
    echo "   ⚠️  Porta 30004 ancora in uso (qualcosa sta usando RTDE)"
    echo "   💡 Verifica sul robot se ci sono programmi attivi"
else
    echo "   ✅ Porta 30004 libera"
fi
echo

echo "4. Verifica errore robot C218A1..."
echo "   Errore C218A1 dal robot - verifica sul teach pendant cosa significa"
echo "   Potrebbe essere un errore di sicurezza o configurazione"
echo

echo "================================================================================"
echo "SOLUZIONE RACCOMANDATA"
echo "================================================================================"
echo
echo "Il driver ROS2 sta crasando. Usa socket diretto invece:"
echo
echo "1. NON avviare driver ROS2 (causa problemi)"
echo "2. Avvia SOLO web interface con socket:"
echo
echo "   cd ~/MekoAiAccelerator"
echo "   export UR_ROBOT_IP=192.168.10.194"
echo "   export WEB_PORT=8081"
echo "   python3 -m remote_ur_control.web_interface"
echo
echo "3. Sul teach pendant: avvia programma normale (NON External Control)"
echo "4. Metti in PLAYING"
echo "5. Controlla robot via web interface (usa socket diretto)"
echo
echo "================================================================================"










