#!/bin/bash

# Script per liberare la porta 50002 e riavviare il driver ROS2

echo "=================================================================================="
echo "🔧 LIBERA PORTA 50002 E RIAVVIA DRIVER ROS2"
echo "=================================================================================="
echo ""

# 1. Trova processo che usa porta 50002
echo "1. Cercando processi che usano porta 50002..."

PID_50002=""

# Prova con lsof
if command -v lsof > /dev/null 2>&1; then
    PID_50002=$(lsof -ti :50002 2>/dev/null | head -1)
    if [ -n "$PID_50002" ]; then
        echo "   ✅ Trovato processo con lsof: PID $PID_50002"
        ps aux | grep "$PID_50002" | grep -v grep | head -1
    fi
fi

# Se non trovato, prova con fuser
if [ -z "$PID_50002" ] && command -v fuser > /dev/null 2>&1; then
    PID_50002=$(fuser 50002/tcp 2>/dev/null | awk '{print $1}' | head -1)
    if [ -n "$PID_50002" ]; then
        echo "   ✅ Trovato processo con fuser: PID $PID_50002"
        ps aux | grep "$PID_50002" | grep -v grep | head -1
    fi
fi

# Se ancora non trovato, prova con netstat
if [ -z "$PID_50002" ]; then
    PID_50002=$(netstat -tulnp 2>/dev/null | grep ":50002 " | awk '{print $7}' | cut -d'/' -f1 | head -1)
    if [ -n "$PID_50002" ] && [ "$PID_50002" != "-" ]; then
        echo "   ✅ Trovato processo con netstat: PID $PID_50002"
        ps aux | grep "$PID_50002" | grep -v grep | head -1
    fi
fi

# Se ancora non trovato, prova con ss
if [ -z "$PID_50002" ]; then
    PID_50002=$(ss -tulnp 2>/dev/null | grep ":50002 " | grep -oP 'pid=\K[0-9]+' | head -1)
    if [ -n "$PID_50002" ]; then
        echo "   ✅ Trovato processo con ss: PID $PID_50002"
        ps aux | grep "$PID_50002" | grep -v grep | head -1
    fi
fi

if [ -z "$PID_50002" ]; then
    echo "   ⚠️  Nessun processo trovato con strumenti standard"
    echo "   Potrebbe essere un processo zombie o un problema di permessi"
fi

echo ""

# 2. Kill processo driver ROS2 esistente
echo "2. Fermando driver ROS2 esistente..."
pkill -9 -f ur_ros2_control_node 2>/dev/null || true
sleep 2

# 3. Kill processo sulla porta 50002 se trovato
if [ -n "$PID_50002" ]; then
    echo "3. Kill processo sulla porta 50002 (PID: $PID_50002)..."
    kill -9 "$PID_50002" 2>/dev/null || true
    sleep 1
    
    # Verifica che sia morto
    if ps -p "$PID_50002" > /dev/null 2>&1; then
        echo "   ⚠️  Processo ancora vivo, provo kill -9..."
        kill -9 "$PID_50002" 2>/dev/null || true
        sleep 1
    fi
    
    if ! ps -p "$PID_50002" > /dev/null 2>&1; then
        echo "   ✅ Processo killato"
    else
        echo "   ❌ Impossibile killare processo $PID_50002"
        echo "   Potrebbe richiedere permessi root"
    fi
else
    echo "3. Nessun processo specifico da killare sulla porta 50002"
fi

echo ""

# 4. Verifica porta libera
echo "4. Verifica porta 50002 libera..."
sleep 1
if netstat -tuln 2>/dev/null | grep -q ":50002 "; then
    echo "   ⚠️  Porta 50002 ancora in uso!"
    echo "   Processi rimanenti:"
    netstat -tulnp 2>/dev/null | grep ":50002 " | sed 's/^/   /'
else
    echo "   ✅ Porta 50002 LIBERA"
fi

echo ""

# 5. Verifica che driver ROS2 sia fermo
echo "5. Verifica driver ROS2 fermo..."
if pgrep -f "ur_ros2_control_node" > /dev/null; then
    echo "   ⚠️  Driver ROS2 ancora attivo, killo..."
    pkill -9 -f ur_ros2_control_node
    sleep 2
fi

if ! pgrep -f "ur_ros2_control_node" > /dev/null; then
    echo "   ✅ Driver ROS2 fermo"
else
    echo "   ❌ Impossibile fermare driver ROS2"
    exit 1
fi

echo ""
echo "=================================================================================="
echo "✅ PORTA 50002 LIBERATA"
echo "=================================================================================="
echo ""
echo "Ora puoi avviare il driver ROS2 con:"
echo "  ./avvia_driver_ros2.sh"
echo ""
echo "OPPURE dalla web interface:"
echo "  Clicca '▶️ Avvia Driver ROS2'"
echo ""







