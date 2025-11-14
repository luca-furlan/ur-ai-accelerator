#!/bin/bash
# Script per trasferire il codice sull'AI Accelerator e testarlo

AI_ACCELERATOR_IP="192.168.10.191"
AI_ACCELERATOR_USER="lab"
REMOTE_DIR="~/MekoAiAccelerator"

echo "=========================================="
echo "Deploy su AI Accelerator"
echo "=========================================="
echo "IP: $AI_ACCELERATOR_IP"
echo "User: $AI_ACCELERATOR_USER"
echo ""

# Crea directory remota
echo "1. Creazione directory remota..."
ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "mkdir -p $REMOTE_DIR/remote_ur_control"

# Trasferisci file
echo "2. Trasferimento file..."
scp -r remote_ur_control/* $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/remote_ur_control/
scp ros2_bridge_fixed.py $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/
scp setup.py $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/ 2>/dev/null || true

echo ""
echo "3. Installazione dipendenze su AI Accelerator..."
ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP << 'ENDSSH'
cd ~/MekoAiAccelerator
python3 -m pip install --user flask || pip3 install --user flask
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080
echo "Dipendenze installate"
ENDSSH

echo ""
echo "✅ Deploy completato!"
echo ""
echo "Per avviare la web interface sull'AI Accelerator:"
echo "  ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo "  cd ~/MekoAiAccelerator"
echo "  export UR_ROBOT_IP=192.168.10.194"
echo "  export WEB_HOST=0.0.0.0"
echo "  export WEB_PORT=8080"
echo "  python3 -m remote_ur_control.web_interface"
echo ""
echo "Poi apri: http://$AI_ACCELERATOR_IP:8080"


