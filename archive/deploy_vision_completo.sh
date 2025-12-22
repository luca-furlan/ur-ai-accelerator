#!/bin/bash

# ========================================
# DEPLOY COMPLETO SISTEMA VISION + ROBOT
# Tutto su AI Accelerator
# ========================================

set -e

# Configurazione (modifica questi valori)
AI_ACCELERATOR_IP="${AI_ACCELERATOR_IP:-192.168.1.100}"
AI_ACCELERATOR_USER="${AI_ACCELERATOR_USER:-user}"
AI_ACCELERATOR_PATH="${AI_ACCELERATOR_PATH:-~/MekoAiAccelerator}"

echo "=========================================="
echo "DEPLOY COMPLETO → AI ACCELERATOR"
echo "=========================================="
echo ""
echo "Target: $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo "Path: $AI_ACCELERATOR_PATH"
echo ""

read -p "Continuare? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Deploy annullato"
    exit 0
fi

# ========================================
# 1. Crea directory remota
# ========================================
echo ""
echo "[1/6] Creazione directory remota..."
ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "mkdir -p $AI_ACCELERATOR_PATH/remote_ur_control"
echo "✅ Directory creata"

# ========================================
# 2. Trasferisci nodi ROS2 vision
# ========================================
echo ""
echo "[2/6] Trasferimento nodi vision..."
scp vision_yolo_detector.py \
    moveit_vision_controller.py \
    vision_robot_coordinator.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/
echo "✅ Nodi vision trasferiti"

# ========================================
# 3. Trasferisci web interface + API
# ========================================
echo ""
echo "[3/6] Trasferimento web interface..."

# Web interface esistente
scp remote_ur_control/web_interface.py \
    remote_ur_control/remote_ur_controller.py \
    remote_ur_control/rtde_manager.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/remote_ur_control/

# Vision web API
scp remote_ur_control/vision_web_api.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/remote_ur_control/

# Script integrazione
scp integra_vision_web_interface.py \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

echo "✅ Web interface trasferita"

# ========================================
# 4. Trasferisci launch files e script
# ========================================
echo ""
echo "[4/6] Trasferimento launch files..."
scp launch_vision_robot_system.py \
    avvia_vision_robot_system.sh \
    ferma_vision_robot_system.sh \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Script avvio completo
cat > /tmp/avvia_sistema_completo.sh << 'EOF'
#!/bin/bash

# Avvia sistema completo: Vision + Web Interface

echo "=========================================="
echo "AVVIO SISTEMA COMPLETO"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Aggiungi path
export PYTHONPATH=$(pwd):$PYTHONPATH

# 1. Avvia vision system in background
echo "[1/2] Avvio vision system..."
bash avvia_vision_robot_system.sh > /tmp/vision_system.log 2>&1 &
VISION_PID=$!

# Attendi avvio
sleep 5

# 2. Integra vision API in web interface (se non già fatto)
echo "[2/2] Setup web interface..."
if ! grep -q "vision_web_api" remote_ur_control/web_interface.py; then
    echo "→ Integrazione Vision API..."
    python3 integra_vision_web_interface.py
fi

# 3. Avvia web interface
echo ""
echo "✅ Avvio web interface..."
echo ""
echo "=========================================="
echo "Sistema ONLINE!"
echo "=========================================="
echo ""
echo "Accedi da browser:"
echo "  http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "Vision system PID: $VISION_PID"
echo "Log: /tmp/vision_system.log"
echo ""
echo "Per fermare: Ctrl+C o ./ferma_sistema_completo.sh"
echo "=========================================="
echo ""

# Avvia web interface (foreground)
export UR_ROBOT_IP=192.168.10.194
python3 -m remote_ur_control.web_interface
EOF

scp /tmp/avvia_sistema_completo.sh \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

# Script stop
cat > /tmp/ferma_sistema_completo.sh << 'EOF'
#!/bin/bash

echo "Fermando sistema completo..."

# Ferma web interface
pkill -f "remote_ur_control.web_interface"

# Ferma vision system
bash ferma_vision_robot_system.sh 2>/dev/null

echo "✅ Sistema fermato"
EOF

scp /tmp/ferma_sistema_completo.sh \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/

echo "✅ Script trasferiti"

# ========================================
# 5. Trasferisci test e docs
# ========================================
echo ""
echo "[5/6] Trasferimento documentazione..."
scp test_vision_system.py \
    README_VISION_ROBOT.md \
    QUICK_START_VISION.md \
    VISION_SYSTEM_COMPLETE.md \
    ARCHITETTURA_DEPLOY.md \
    $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$AI_ACCELERATOR_PATH/
echo "✅ Documentazione trasferita"

# ========================================
# 6. Setup e installazione automatica
# ========================================
echo ""
echo "[6/6] Setup e installazione su AI Accelerator..."

ssh -t $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP << 'ENDSSH'
cd ~/MekoAiAccelerator

echo ""
echo "🔧 INSTALLAZIONE AUTOMATICA DIPENDENZE"
echo ""

# Permessi eseguibili
chmod +x *.sh *.py

# Esegui script installazione automatica
if [ -f "installa_dipendenze_vision.sh" ]; then
    bash installa_dipendenze_vision.sh
else
    echo "⚠️  Script installazione non trovato, installazione manuale..."
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    
    # Python packages
    echo "→ Installazione Python packages..."
    pip3 install -q ultralytics opencv-python numpy flask requests Pillow --user
    
    # ROS2 packages
    echo "→ Installazione ROS2 packages..."
    sudo apt install -y ros-humble-cv-bridge ros-humble-vision-msgs
    
    echo "✅ Installazione base completata"
fi

echo ""
echo "✅ Setup completato"
ENDSSH

echo "✅ Setup remoto completato"

# ========================================
# Info finale
# ========================================
echo ""
echo "=========================================="
echo "✅ DEPLOY COMPLETATO!"
echo "=========================================="
echo ""
echo "Prossimi passi:"
echo ""
echo "1. SSH su AI Accelerator:"
echo "   ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo ""
echo "2. Avvia sistema completo:"
echo "   cd $AI_ACCELERATOR_PATH"
echo "   bash avvia_sistema_completo.sh"
echo ""
echo "3. Da Windows, apri browser:"
echo "   http://$AI_ACCELERATOR_IP:5000"
echo ""
echo "Per fermare sistema:"
echo "   bash ferma_sistema_completo.sh"
echo ""
echo "=========================================="

