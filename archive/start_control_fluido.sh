#!/bin/bash
# Script per avviare il controllo fluido del robot UR con joystick simulato
# Da eseguire SULL'AI ACCELERATOR

set -e

echo "=========================================="
echo "AVVIO CONTROLLO FLUIDO ROBOT UR"
echo "=========================================="
echo ""

# Configurazione
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Verifica directory
if [ ! -d ~/MekoAiAccelerator ]; then
    echo "❌ Directory ~/MekoAiAccelerator non trovata!"
    echo "   Esegui prima il deploy: bash deploy_to_ai_accelerator_complete.sh"
    exit 1
fi

cd ~/MekoAiAccelerator

# Verifica file necessari
if [ ! -f remote_ur_control/web_interface.py ]; then
    echo "❌ web_interface.py non trovato!"
    exit 1
fi

# Source ROS2 se disponibile
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble caricato"
    
    # Source workspace se presente
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
        echo "✅ ROS2 workspace caricato"
    fi
else
    echo "⚠️  ROS2 non trovato - userà controllo socket diretto"
fi

# Verifica dipendenze Python
echo ""
echo "Verifica dipendenze..."
if ! python3 -c "import flask" 2>/dev/null; then
    echo "⚠️  Flask non trovato - installazione..."
    python3 -m pip install --user flask flask-cors || pip3 install --user flask flask-cors
fi

# Verifica connessione robot
echo ""
echo "Verifica connessione robot ($UR_ROBOT_IP)..."
if ping -c 1 $UR_ROBOT_IP > /dev/null 2>&1; then
    echo "✅ Robot raggiungibile"
else
    echo "❌ Robot NON raggiungibile - verifica connessione di rete"
    exit 1
fi

# Verifica porta robot
if timeout 2 bash -c "</dev/tcp/$UR_ROBOT_IP/30002" 2>/dev/null; then
    echo "✅ Porta 30002 (URScript) raggiungibile"
else
    echo "⚠️  Porta 30002 non raggiungibile - il robot potrebbe non essere pronto"
fi

echo ""
echo "=========================================="
echo "AVVIO WEB INTERFACE CON JOYSTICK"
echo "=========================================="
echo ""
echo "🌐 Web Interface sarà disponibile su:"
echo "   http://$(hostname -I | awk '{print $1}'):$WEB_PORT"
echo "   oppure"
echo "   http://192.168.10.191:$WEB_PORT"
echo ""
echo "🎮 Il joystick simulato è nella pagina web!"
echo ""
echo "Premi CTRL+C per fermare"
echo ""

# Avvia web interface
python3 -m remote_ur_control.web_interface

