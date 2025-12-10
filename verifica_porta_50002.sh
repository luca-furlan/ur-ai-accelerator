#!/bin/bash

# Script per verificare se la porta 50002 è effettivamente aperta e in ascolto

echo "=================================================================================="
echo "🔍 VERIFICA PORTA 50002"
echo "=================================================================================="
echo ""

# 1. Verifica se il driver ROS2 è attivo
echo "1. Driver ROS2:"
if pgrep -f "ur_ros2_control_node" > /dev/null; then
    PID=$(pgrep -f "ur_ros2_control_node" | head -1)
    echo "   ✅ Driver ROS2 ATTIVO (PID: $PID)"
else
    echo "   ❌ Driver ROS2 NON attivo"
    echo ""
    echo "   Avvia prima il driver ROS2 con:"
    echo "   ./avvia_driver_ros2.sh"
    exit 1
fi

echo ""

# 2. Verifica porta 50002 con netstat
echo "2. Porta 50002 (netstat):"
if netstat -tuln 2>/dev/null | grep -q ":50002 "; then
    echo "   ✅ Porta 50002 APERTA"
    netstat -tuln | grep ":50002 " | head -1
else
    echo "   ❌ Porta 50002 NON aperta"
fi

echo ""

# 3. Verifica porta 50002 con ss (alternativa)
echo "3. Porta 50002 (ss):"
if ss -tuln 2>/dev/null | grep -q ":50002 "; then
    echo "   ✅ Porta 50002 APERTA (confermato da ss)"
    ss -tuln | grep ":50002 " | head -1
else
    echo "   ❌ Porta 50002 NON aperta (ss)"
fi

echo ""

# 4. Prova connessione TCP diretta
echo "4. Test connessione TCP:"
if timeout 2 bash -c "echo > /dev/tcp/127.0.0.1/50002" 2>/dev/null; then
    echo "   ✅ Porta 50002 RAGGIUNGIBILE"
else
    echo "   ❌ Porta 50002 NON raggiungibile"
fi

echo ""

# 5. Verifica log driver per errori
echo "5. Ultimi errori nel log driver:"
if [ -f /tmp/ros2_driver.log ]; then
    echo "   Ultime 30 righe del log:"
    tail -30 /tmp/ros2_driver.log | grep -E "(ERROR|WARN|System successfully|Connected|50002|port)" | tail -10 | sed 's/^/   /'
    
    # Verifica se c'è "System successfully started!"
    if grep -q "System successfully started" /tmp/ros2_driver.log; then
        echo ""
        echo "   ✅ Driver ha completato l'avvio con successo"
    else
        echo ""
        echo "   ⚠️  Driver potrebbe non aver completato l'avvio"
    fi
    
    # Verifica errori di connessione
    if grep -q "connection refused\|Connection refused\|refused" /tmp/ros2_driver.log; then
        echo ""
        echo "   ⚠️  Trovati errori 'connection refused' nel log"
    fi
else
    echo "   ⚠️  File di log non trovato"
fi

echo ""

# 6. Verifica processi che usano la porta 50002
echo "6. Processi che usano porta 50002:"
if command -v lsof > /dev/null 2>&1; then
    if lsof -i :50002 2>/dev/null | grep -v COMMAND; then
        echo "   Processi trovati:"
        lsof -i :50002 2>/dev/null | grep -v COMMAND | sed 's/^/   /'
    else
        echo "   ❌ Nessun processo in ascolto sulla porta 50002"
    fi
else
    echo "   ⚠️  lsof non disponibile, usa netstat/ss"
fi

echo ""
echo "=================================================================================="
echo "💡 DIAGNOSI:"
echo "=================================================================================="
echo ""

# Diagnosi finale
PORT_OPEN=false
if netstat -tuln 2>/dev/null | grep -q ":50002 " || ss -tuln 2>/dev/null | grep -q ":50002 "; then
    PORT_OPEN=true
fi

DRIVER_ACTIVE=false
if pgrep -f "ur_ros2_control_node" > /dev/null; then
    DRIVER_ACTIVE=true
fi

if $DRIVER_ACTIVE && $PORT_OPEN; then
    echo "✅ TUTTO OK: Driver attivo e porta 50002 aperta"
    echo ""
    echo "   Se ancora ricevi 'connection refused' sul Teach Pendant:"
    echo "   1. Verifica IP sul Teach Pendant: 192.168.10.191"
    echo "   2. Verifica Porta sul Teach Pendant: 50002"
    echo "   3. Verifica che il robot sia in Remote Control mode"
    echo "   4. Riavvia il programma External Control sul Teach Pendant"
elif $DRIVER_ACTIVE && ! $PORT_OPEN; then
    echo "❌ PROBLEMA: Driver attivo ma porta 50002 NON aperta"
    echo ""
    echo "   Possibili cause:"
    echo "   1. Driver crashato subito dopo l'avvio"
    echo "   2. Driver non ha completato l'inizializzazione"
    echo "   3. Problema con la configurazione del driver"
    echo ""
    echo "   Soluzione:"
    echo "   1. Controlla il log completo: tail -100 /tmp/ros2_driver.log"
    echo "   2. Ferma e riavvia il driver: pkill -f ur_ros2_control_node && ./avvia_driver_ros2.sh"
elif ! $DRIVER_ACTIVE; then
    echo "❌ PROBLEMA: Driver ROS2 NON attivo"
    echo ""
    echo "   Avvia il driver con: ./avvia_driver_ros2.sh"
fi

echo ""

