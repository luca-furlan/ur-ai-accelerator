#!/bin/bash
# Script da eseguire DIRETTAMENTE sull'AI Accelerator per completare setup
# I file sono già stati trasferiti

set -e

echo "=================================================================================="
echo "COMPLETAMENTO SETUP VISION SYSTEM"
echo "=================================================================================="
echo ""

cd ~/MekoAiAccelerator

# 1. Permessi
echo "[1/5] Configurazione permessi..."
chmod +x *.py 2>/dev/null || true
echo "✅ Permessi configurati"

# 2. Source ROS2
echo ""
echo "[2/5] Configurazione ROS2..."
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
echo "✅ ROS2 configurato"

# 3. Installa ROS2 packages (richiede sudo)
echo ""
echo "[3/5] Installazione ROS2 packages..."
echo "  (Richiede password sudo)"
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-msgs -qq
echo "✅ ROS2 packages installati"

# 4. Integra Vision API
echo ""
echo "[4/5] Integrazione Vision API..."

if ! grep -q "vision_web_api" remote_ur_control/web_interface.py 2>/dev/null; then
    # Backup
    cp remote_ur_control/web_interface.py remote_ur_control/web_interface.py.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
    
    # Usa Python per modificare file
    python3 << 'PYEOF'
import re

with open('remote_ur_control/web_interface.py', 'r') as f:
    content = f.read()

if 'vision_web_api' not in content:
    # Aggiungi import dopo "from flask import"
    import_pattern = r'(from flask import[^\n]+)'
    vision_import = r'\1\n\n# Vision API\ntry:\n    from .vision_web_api import add_vision_routes_to_app\n    VISION_API_AVAILABLE = True\nexcept ImportError:\n    VISION_API_AVAILABLE = False\n    print("⚠️ Vision API non disponibile")'
    content = re.sub(import_pattern, vision_import, content, count=1)
    
    # Aggiungi inizializzazione dopo "app = Flask"
    app_pattern = r'(app = Flask\([^\n]+)'
    vision_init = r'\1\n\n# Initialize Vision API\nif VISION_API_AVAILABLE:\n    try:\n        vision_api = add_vision_routes_to_app(app)\n        print("✅ Vision API integrata")\n    except Exception as e:\n        print(f"⚠️ Errore integrazione Vision API: {e}")'
    content = re.sub(app_pattern, vision_init, content, count=1)
    
    with open('remote_ur_control/web_interface.py', 'w') as f:
        f.write(content)
    print("✅ Vision API integrata")
else:
    print("✅ Vision API già integrata")
PYEOF
else
    echo "✅ Vision API già integrata"
fi

# 5. Crea script avvio
echo ""
echo "[5/5] Creazione script avvio..."

cat > avvia_vision_completo.sh << 'EOFBASH'
#!/bin/bash
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
export PYTHONPATH=$(pwd):$PYTHONPATH
export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}

echo "🚀 Avvio Vision System Completo"
echo ""

# Avvia camera se disponibile
if ros2 pkg list | grep -q orbbec_camera; then
    echo "→ Avvio camera..."
    ros2 launch orbbec_camera gemini_330_series.launch.py > /tmp/camera.log 2>&1 &
    sleep 3
fi

# Avvia vision detector
echo "→ Avvio vision detector..."
python3 vision_yolo_detector.py > /tmp/vision.log 2>&1 &
sleep 2

# Avvia coordinator
echo "→ Avvio coordinator..."
python3 vision_robot_coordinator.py > /tmp/coordinator.log 2>&1 &
sleep 1

echo ""
echo "✅ Vision system avviato!"
echo ""
echo "Log: /tmp/vision.log, /tmp/coordinator.log"
echo ""
echo "Avvio web interface..."
echo ""

python3 -m remote_ur_control.web_interface
EOFBASH

chmod +x avvia_vision_completo.sh
echo "✅ Script avvio creato"

echo ""
echo "=================================================================================="
echo "✅ SETUP COMPLETATO!"
echo "=================================================================================="
echo ""
echo "Per avviare il sistema:"
echo "  ./avvia_vision_completo.sh"
echo ""
echo "Accesso da browser:"
echo "  http://$(hostname -I | awk '{print $1}'):8080"
echo ""
echo "Nuove API disponibili:"
echo "  GET  /api/vision/status"
echo "  GET  /api/vision/detections"
echo "  POST /api/vision/pick_target"
echo ""
echo "=================================================================================="




