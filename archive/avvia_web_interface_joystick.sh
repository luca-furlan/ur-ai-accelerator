#!/bin/bash

# Script per avviare la web interface con joystick per controllo robot
# Configurato per movimenti LENTI e SICURI

# Non usare set -e perché vogliamo gestire gli errori manualmente
set +e

ROBOT_IP="${UR_ROBOT_IP:-192.168.10.194}"
WEB_PORT="${WEB_PORT:-8080}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"

echo "=================================================================================="
echo "🎮 AVVIO WEB INTERFACE CON JOYSTICK - MOVIMENTI LENTI"
echo "=================================================================================="
echo ""
echo "Configurazione:"
echo "  Robot IP: $ROBOT_IP"
echo "  Web Interface: http://$WEB_HOST:$WEB_PORT"
echo "  Velocità joystick: RIDOTTE per movimenti lenti e sicuri"
echo ""

# Verifica che il driver ROS2 sia attivo (solo informativo)
if ! pgrep -f "ur_ros2_control_node" > /dev/null; then
    echo "ℹ️  Driver ROS2 non attivo - puoi avviarlo dalla web interface!"
    echo "   Usa il pulsante '▶️ Avvia Driver ROS2' nel pannello 'Sistema Robot'"
    echo ""
else
    echo "✅ Driver ROS2 attivo"
fi

# Verifica e kill processi esistenti (IMPORTANTE per evitare crash)
echo "🔍 Verifica processi esistenti..."
CURRENT_PID=$$

# Trova processi Python che eseguono web_interface (escludendo questo script)
EXISTING_WEB=$(pgrep -f "python.*web_interface" 2>/dev/null || true)

if [ -n "$EXISTING_WEB" ]; then
    echo "⚠️  Trovati processi web interface esistenti"
    KILLED_COUNT=0
    # Kill solo i processi che NON sono questo script
    for pid in $EXISTING_WEB; do
        # Verifica che il PID non sia questo processo o il suo parent
        if [ "$pid" != "$CURRENT_PID" ] && [ "$pid" != "$PPID" ]; then
            # Verifica che sia effettivamente un processo Python web_interface
            CMD=$(ps -p "$pid" -o cmd= 2>/dev/null || echo "")
            if echo "$CMD" | grep -q "python.*web_interface"; then
                echo "   Terminando processo $pid ($(echo $CMD | cut -c1-60)...)..."
                kill -9 "$pid" 2>/dev/null && KILLED_COUNT=$((KILLED_COUNT + 1)) || true
            fi
        fi
    done
    
    if [ $KILLED_COUNT -gt 0 ]; then
        echo "✅ Terminati $KILLED_COUNT processi web interface"
        sleep 2
    else
        echo "ℹ️  Nessun processo da terminare (tutti sono questo script)"
    fi
else
    echo "✅ Nessun processo web interface esistente trovato"
fi

# Verifica porta libera (dopo kill processi)
sleep 1  # Attendi che i processi vengano killati
if netstat -tuln 2>/dev/null | grep -q ":$WEB_PORT "; then
    echo "⚠️  Porta $WEB_PORT ancora in uso dopo kill processi!"
    echo "   Attendo 3 secondi per rilascio porta..."
    sleep 3
    if netstat -tuln 2>/dev/null | grep -q ":$WEB_PORT "; then
        # Verifica se è questo processo stesso che usa la porta
        PORT_PID=$(lsof -ti :$WEB_PORT 2>/dev/null | head -1 || true)
        if [ -n "$PORT_PID" ] && [ "$PORT_PID" != "$CURRENT_PID" ] && [ "$PORT_PID" != "$PPID" ]; then
            echo "⚠️  Porta $WEB_PORT ancora occupata da processo $PORT_PID"
            echo "   Provo a killarlo..."
            kill -9 "$PORT_PID" 2>/dev/null || true
            sleep 2
        fi
        
        # Verifica finale
        if netstat -tuln 2>/dev/null | grep -q ":$WEB_PORT "; then
            echo "⚠️  Porta $WEB_PORT ancora occupata, ma continuo comunque..."
            echo "   (Potrebbe essere un problema temporaneo)"
        else
            echo "✅ Porta $WEB_PORT ora libera"
        fi
    else
        echo "✅ Porta $WEB_PORT ora libera"
    fi
fi

# Source ROS2 (IMPORTANTE per abilitare ROS2 bridge)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble configurato"
else
    echo "⚠️  ROS2 Humble non trovato in /opt/ros/humble/setup.bash"
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ Workspace ROS2 configurato"
fi

# Esporta variabili ambiente
export UR_ROBOT_IP="$ROBOT_IP"
export WEB_HOST="$WEB_HOST"
export WEB_PORT="$WEB_PORT"
export WEB_DEBUG="0"

echo "✅ Avvio web interface..."
echo ""
echo "📋 ISTRUZIONI:"
echo "   1. Apri il browser su: http://$(hostname -I | awk '{print $1}'):$WEB_PORT"
echo "   2. Se il driver ROS2 non è attivo, clicca '▶️ Avvia Driver ROS2' nel pannello 'Sistema Robot'"
echo "   3. Clicca '🔄 Switch Controller' per attivare il controller"
echo "   4. Usa i joystick virtuali per muovere il robot"
echo "   5. Velocità sono RIDOTTE per movimenti lenti e sicuri"
echo "   6. Premi 'Emergency Stop' per fermare immediatamente"
echo ""
echo "⚠️  SICUREZZA:"
echo "   - Assicurati che l'area di lavoro sia libera"
echo "   - Tieni pronto l'e-stop fisico del robot"
echo "   - Inizia con movimenti piccoli"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

# Avvia web interface con auto-restart
cd ~/MekoAiAccelerator

# Flag per indicare interruzione volontaria (CTRL+C)
USER_INTERRUPTED=0

# Funzione per killare processi prima di uscire
cleanup() {
    # Evita loop infinito: se già in cleanup, esci immediatamente
    if [ $USER_INTERRUPTED -eq 1 ]; then
        exit 0
    fi
    
    USER_INTERRUPTED=1
    echo ""
    echo "🛑 Fermando web interface..."
    
    # Kill processi Python web_interface (non questo script)
    for pid in $(pgrep -f "python.*web_interface" 2>/dev/null || true); do
        if [ "$pid" != "$CURRENT_PID" ] && [ "$pid" != "$PPID" ]; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    
    # Attendi un attimo per terminazione graceful
    sleep 1
    
    # Se ancora attivi, kill forzato
    for pid in $(pgrep -f "python.*web_interface" 2>/dev/null || true); do
        if [ "$pid" != "$CURRENT_PID" ] && [ "$pid" != "$PPID" ]; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    
    echo "✅ Web interface fermata"
    exit 0
}

# Trap per SIGINT (CTRL+C) e SIGTERM
trap cleanup SIGINT SIGTERM

# Loop con auto-restart (max 5 tentativi)
MAX_RESTARTS=5
RESTART_COUNT=0

while [ $RESTART_COUNT -lt $MAX_RESTARTS ] && [ $USER_INTERRUPTED -eq 0 ]; do
    if [ $RESTART_COUNT -gt 0 ]; then
        echo ""
        echo "⚠️  Web interface crashato. Riavvio tentativo $RESTART_COUNT/$MAX_RESTARTS..."
        echo "   Pulizia processi esistenti..."
        # Kill solo processi Python web_interface (non questo script)
        for pid in $(pgrep -f "python.*web_interface" 2>/dev/null || true); do
            if [ "$pid" != "$CURRENT_PID" ] && [ "$pid" != "$PPID" ]; then
                kill -9 "$pid" 2>/dev/null || true
            fi
        done
        sleep 2
    fi
    
    # Controlla se l'utente ha premuto CTRL+C prima di avviare
    if [ $USER_INTERRUPTED -eq 1 ]; then
        break
    fi
    
    echo "🚀 Avvio web interface (tentativo $((RESTART_COUNT + 1))/$MAX_RESTARTS)..."
    python3 -m remote_ur_control.web_interface
    
    EXIT_CODE=$?
    
    # Se l'utente ha premuto CTRL+C durante l'esecuzione, esci
    if [ $USER_INTERRUPTED -eq 1 ]; then
        break
    fi
    
    if [ $EXIT_CODE -eq 0 ]; then
        echo "✅ Web interface terminato normalmente"
        break
    elif [ $EXIT_CODE -eq 130 ] || [ $EXIT_CODE -eq 143 ]; then
        # Exit code 130 = SIGINT (CTRL+C), 143 = SIGTERM
        echo "✅ Web interface terminato dall'utente (CTRL+C)"
        break
    else
        RESTART_COUNT=$((RESTART_COUNT + 1))
        if [ $RESTART_COUNT -ge $MAX_RESTARTS ]; then
            echo ""
            echo "❌ Web interface crashato $MAX_RESTARTS volte. Fermo auto-restart."
            echo "   Verifica i log per errori:"
            echo "   - Memoria disponibile: free -h"
            echo "   - Processi ROS2: pgrep -f ur_ros2_control_node"
            echo "   - Log sistema: dmesg | tail -20"
            exit 1
        fi
        echo "   Attendo 5 secondi prima di riavviare..."
        sleep 5
    fi
done

