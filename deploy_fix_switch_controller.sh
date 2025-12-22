#!/bin/bash
# Script bash per deploy fix switch_controller
# Copia i file modificati sulla macchina remota

AI_ACCELERATOR_IP="${AI_ACCELERATOR_IP:-192.168.10.191}"
AI_ACCELERATOR_USER="${AI_ACCELERATOR_USER:-lab}"
AI_ACCELERATOR_PATH="${AI_ACCELERATOR_PATH:-~/MekoAiAccelerator}"

echo "=================================================================================="
echo "DEPLOY FIX SWITCH CONTROLLER"
echo "=================================================================================="
echo ""
echo "Target: $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo "Path: $AI_ACCELERATOR_PATH"
echo ""

# Test connessione
echo "[1/3] Test connessione SSH..."
if ! ssh -o ConnectTimeout=5 "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "echo OK" > /dev/null 2>&1; then
    echo "❌ Errore connessione SSH"
    exit 1
fi
echo "✅ Connessione SSH OK"
echo ""

# Trasferimento file
echo "[2/3] Trasferimento file modificati..."

echo "  - switch_controller.py..."
if scp switch_controller.py "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/" 2>&1; then
    echo "    ✅ switch_controller.py copiato"
else
    echo "    ❌ Errore copia switch_controller.py"
    exit 1
fi

echo "  - remote_ur_control/web_interface.py..."
if scp remote_ur_control/web_interface.py "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/remote_ur_control/" 2>&1; then
    echo "    ✅ web_interface.py copiato"
else
    echo "    ❌ Errore copia web_interface.py"
    exit 1
fi

echo ""
echo "[3/3] Riavvio web interface..."

# Ferma web interface esistente
ssh "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "pkill -f 'web_interface' || true" 2>/dev/null
sleep 2

# Riavvia web interface
echo "  Avvio web interface in background..."
ssh "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" << 'ENDSSH'
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
echo $!
ENDSSH

sleep 3

# Verifica
PID=$(ssh "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "pgrep -f 'web_interface' | head -1" 2>/dev/null)
if [ -n "$PID" ]; then
    echo "  ✅ Web interface avviata (PID: $PID)"
else
    echo "  ⚠️  Web interface potrebbe non essere avviata. Verifica manualmente."
    echo "     Log: ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP 'tail -50 /tmp/web_interface.log'"
fi

echo ""
echo "=================================================================================="
echo "✅ DEPLOY COMPLETATO"
echo "=================================================================================="
echo ""
echo "File copiati:"
echo "  - switch_controller.py"
echo "  - remote_ur_control/web_interface.py"
echo ""
echo "Web interface:"
echo "  http://$AI_ACCELERATOR_IP:8080"
echo ""
echo "Per vedere i log:"
echo "  ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP 'tail -f /tmp/web_interface.log'"
echo ""

