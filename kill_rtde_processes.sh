#!/bin/bash
# Script per verificare e killare altri processi RTDE che potrebbero interferire

echo "🔍 Verifica processi RTDE attivi..."

# Escludi questo script stesso e il processo padre
SCRIPT_PID=$$
PARENT_PID=$PPID
EXCLUDE_PIDS="$SCRIPT_PID $PARENT_PID"

# Lista di pattern da cercare (escludendo kill_rtde_processes.sh)
RTDE_PATTERNS=(
    "ur_rtde"
    "ur_robot_driver"
    "ur_ros2_control_node"
)

KILLED_COUNT=0

for pattern in "${RTDE_PATTERNS[@]}"; do
    # Trova processi che corrispondono al pattern, escludendo questo script
    PIDS=$(pgrep -f "$pattern" 2>/dev/null || true)
    
    if [ -n "$PIDS" ]; then
        echo "   Trovati processi con pattern '$pattern':"
        for pid in $PIDS; do
            # Salta questo script stesso e il processo padre
            if echo "$EXCLUDE_PIDS" | grep -q "\b$pid\b"; then
                continue
            fi
            
            # Verifica che non sia questo script
            CMD=$(ps -p "$pid" -o cmd= 2>/dev/null || echo "")
            if echo "$CMD" | grep -q "kill_rtde_processes.sh"; then
                continue
            fi
            
            # Mostra info processo
            echo "     PID $pid: $(echo $CMD | cut -c1-80)"
            
            # Kill processo
            if kill -9 "$pid" 2>/dev/null; then
                echo "       ✅ Killato"
                KILLED_COUNT=$((KILLED_COUNT + 1))
            else
                echo "       ⚠️  Impossibile killare (potrebbe essere già morto)"
            fi
        done
    fi
done

# Cerca anche processi RTDE generici ma escludi questo script
RTDE_PIDS=$(pgrep -f "rtde" 2>/dev/null | grep -v "^$SCRIPT_PID$" | grep -v "^$PARENT_PID$" || true)

if [ -n "$RTDE_PIDS" ]; then
    echo "   Trovati altri processi RTDE:"
    for pid in $RTDE_PIDS; do
        CMD=$(ps -p "$pid" -o cmd= 2>/dev/null || echo "")
        # Salta questo script
        if echo "$CMD" | grep -q "kill_rtde_processes.sh"; then
            continue
        fi
        # Salta se è già stato killato sopra
        if echo "$CMD" | grep -qE "(ur_rtde|ur_robot_driver|ur_ros2_control_node)"; then
            continue
        fi
        
        echo "     PID $pid: $(echo $CMD | cut -c1-80)"
        if kill -9 "$pid" 2>/dev/null; then
            echo "       ✅ Killato"
            KILLED_COUNT=$((KILLED_COUNT + 1))
        fi
    done
fi

if [ $KILLED_COUNT -eq 0 ]; then
    echo "✅ Nessun processo RTDE trovato da killare"
else
    echo "✅ Killati $KILLED_COUNT processi RTDE"
fi

# Attendi che i processi vengano completamente terminati
sleep 2

# Verifica porta 50002
echo ""
echo "🔍 Verifica porta 50002..."
PORT_PIDS=$(lsof -ti :50002 2>/dev/null || fuser 50002/tcp 2>/dev/null | awk '{print $1}' || true)

if [ -n "$PORT_PIDS" ]; then
    echo "   Porta 50002 occupata da: $PORT_PIDS"
    for pid in $PORT_PIDS; do
        if kill -9 "$pid" 2>/dev/null; then
            echo "     ✅ Killato processo $pid che occupava porta 50002"
            KILLED_COUNT=$((KILLED_COUNT + 1))
        fi
    done
    sleep 1
else
    echo "   ✅ Porta 50002 libera"
fi

exit 0

