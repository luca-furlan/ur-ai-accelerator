#!/bin/bash
# Watchdog per web interface - monitora e riavvia automaticamente se si blocca

cd ~/MekoAiAccelerator || exit 1

PID_FILE="/tmp/web_interface.pid"
LOG_FILE="/tmp/web_interface_watchdog.log"
CHECK_INTERVAL=15  # Controlla ogni 15 secondi
MAX_RESTART_ATTEMPTS=5
RESTART_COOLDOWN=60  # Attendi 60 secondi tra riavvii

function log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

function check_process() {
    local pid=$1
    if [ -z "$pid" ] || [ "$pid" = "0" ]; then
        return 1
    fi
    
    # Verifica che il processo esista
    if ! kill -0 "$pid" 2>/dev/null; then
        return 1
    fi
    
    # Verifica che il processo risponda (porta in ascolto)
    local port=${WEB_PORT:-8080}
    if ! timeout 2 bash -c "echo > /dev/tcp/127.0.0.1/$port" 2>/dev/null; then
        log "WARN: Processo $pid esiste ma porta $port non risponde"
        return 1
    fi
    
    return 0
}

function restart_web_interface() {
    log "RIAVVIO WEB INTERFACE..."
    
    # Kill processi esistenti
    pkill -f web_interface 2>/dev/null
    sleep 2
    
    # Source ROS2
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash 2>/dev/null || true
    fi
    
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}
    export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
    export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}
    export WEB_HOST=${WEB_HOST:-0.0.0.0}
    export WEB_PORT=${WEB_PORT:-8080}
    
    # Libera porta
    fuser -k ${WEB_PORT}/tcp 2>/dev/null || true
    sleep 1
    
    # Avvia nuovo processo
    nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
    local new_pid=$!
    
    # Attendi che il processo si avvii
    sleep 3
    
    # Verifica che sia attivo
    if check_process "$new_pid"; then
        echo "$new_pid" > "$PID_FILE"
        log "OK: Web interface riavviato (PID: $new_pid)"
        return 0
    else
        log "ERROR: Riavvio fallito"
        return 1
    fi
}

# Loop principale
log "Watchdog web interface avviato"
restart_count=0
last_restart_time=0

while true; do
    # Leggi PID corrente
    if [ -f "$PID_FILE" ]; then
        current_pid=$(cat "$PID_FILE" 2>/dev/null)
    else
        current_pid=""
    fi
    
    # Verifica processo
    if check_process "$current_pid"; then
        # Processo OK
        restart_count=0
        sleep "$CHECK_INTERVAL"
        continue
    fi
    
    # Processo non OK - verifica se possiamo riavviare
    local now=$(date +%s)
    local time_since_restart=$((now - last_restart_time))
    
    if [ $time_since_restart -lt $RESTART_COOLDOWN ]; then
        log "WARN: Troppo presto per riavviare (cooldown: $((RESTART_COOLDOWN - time_since_restart))s rimanenti)"
        sleep "$CHECK_INTERVAL"
        continue
    fi
    
    if [ $restart_count -ge $MAX_RESTART_ATTEMPTS ]; then
        log "ERROR: Raggiunto limite massimo riavvii ($MAX_RESTART_ATTEMPTS). Watchdog si ferma."
        exit 1
    fi
    
    # Riavvia
    restart_count=$((restart_count + 1))
    last_restart_time=$now
    
    if restart_web_interface; then
        restart_count=0  # Reset se riavvio riuscito
    fi
    
    sleep "$CHECK_INTERVAL"
done
