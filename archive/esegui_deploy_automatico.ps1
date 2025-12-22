# PowerShell script per eseguire deploy automatico su AI Accelerator
# Esegue TUTTO automaticamente senza intervento utente

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$AI_ACCELERATOR_PATH = "~/MekoAiAccelerator"

Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host "AUTOMATIC DEPLOY TO AI ACCELERATOR" -ForegroundColor Cyan
Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Target: $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Yellow
Write-Host ""

# Verifica SSH disponibile
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "❌ SSH non disponibile. Installa OpenSSH o usa Git Bash." -ForegroundColor Red
    exit 1
}

Write-Host "[1/5] Test connessione SSH..." -ForegroundColor Yellow
$sshTest = ssh -o ConnectTimeout=5 -o BatchMode=yes "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "echo OK" 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "⚠️  SSH richiede password. Configura SSH key per accesso automatico:" -ForegroundColor Yellow
    Write-Host "   ssh-copy-id $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Oppure inserisci password quando richiesta..." -ForegroundColor Yellow
}

Write-Host "[2/5] Creazione directory remota..." -ForegroundColor Yellow
ssh "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "mkdir -p $AI_ACCELERATOR_PATH/remote_ur_control" 2>&1 | Out-Null

Write-Host "[3/5] Trasferimento file vision..." -ForegroundColor Yellow
scp vision_yolo_detector.py "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:$AI_ACCELERATOR_PATH/" 2>&1
scp vision_robot_coordinator.py "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:$AI_ACCELERATOR_PATH/" 2>&1
scp remote_ur_control/vision_web_api.py "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:$AI_ACCELERATOR_PATH/remote_ur_control/" 2>&1

Write-Host "[4/5] Esecuzione setup automatico su AI Accelerator..." -ForegroundColor Yellow

# Crea e esegue script di setup remoto
$setupScript = @"
#!/bin/bash
set -e
cd ~/MekoAiAccelerator

echo "=================================================================================="
echo "SETUP AUTOMATICO VISION SYSTEM"
echo "=================================================================================="
echo ""

# 1. Permessi
chmod +x *.py 2>/dev/null || true
echo "✅ Permessi configurati"

# 2. Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

# 3. Installa dipendenze Python
echo ""
echo "→ Installazione dipendenze Python..."
pip3 install ultralytics opencv-python numpy flask requests --user -q 2>&1 | grep -v "already satisfied" || true
echo "✅ Python packages installati"

# 4. Installa ROS2 packages
echo ""
echo "→ Installazione ROS2 packages..."
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-msgs -qq 2>&1 | grep -v "already" || true
echo "✅ ROS2 packages installati"

# 5. Integra Vision API
echo ""
echo "→ Integrazione Vision API..."
if ! grep -q "vision_web_api" remote_ur_control/web_interface.py 2>/dev/null; then
    # Backup
    cp remote_ur_control/web_interface.py remote_ur_control/web_interface.py.backup.$(date +%Y%m%d_%H%M%S) 2>/dev/null || true
    
    # Aggiungi import dopo "from flask import"
    sed -i '/from flask import/a\\n# Vision API\\ntry:\\n    from .vision_web_api import add_vision_routes_to_app\\n    VISION_API_AVAILABLE = True\\nexcept ImportError:\\n    VISION_API_AVAILABLE = False\\n    print(\"⚠️ Vision API non disponibile\")' remote_ur_control/web_interface.py
    
    # Aggiungi inizializzazione dopo "app = Flask"
    sed -i '/app = Flask/a\\n# Initialize Vision API\\nif VISION_API_AVAILABLE:\\n    try:\\n        vision_api = add_vision_routes_to_app(app)\\n        print(\"✅ Vision API integrata\")\\n    except Exception as e:\\n        print(f\"⚠️ Errore integrazione Vision API: {e}\")' remote_ur_control/web_interface.py
    
    echo "✅ Vision API integrata"
else
    echo "✅ Vision API già integrata"
fi

# 6. Crea script avvio
echo ""
echo "→ Creazione script avvio..."
cat > avvia_vision_completo.sh << 'EOFBASH'
#!/bin/bash
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
export PYTHONPATH=\$(pwd):\$PYTHONPATH
export UR_ROBOT_IP=\${UR_ROBOT_IP:-192.168.10.194}

# Avvia camera se disponibile
if ros2 pkg list | grep -q orbbec_camera; then
    ros2 launch orbbec_camera gemini_330_series.launch.py > /tmp/camera.log 2>&1 &
    sleep 3
fi

# Avvia vision detector
python3 vision_yolo_detector.py > /tmp/vision.log 2>&1 &
sleep 2

# Avvia coordinator
python3 vision_robot_coordinator.py > /tmp/coordinator.log 2>&1 &
sleep 1

echo "✅ Vision system avviato!"
echo "Avvio web interface..."
python3 -m remote_ur_control.web_interface
EOFBASH

chmod +x avvia_vision_completo.sh
echo "✅ Script avvio creato"

echo ""
echo "=================================================================================="
echo "✅ SETUP COMPLETATO!"
echo "=================================================================================="
echo ""
echo "Per avviare:"
echo "  ./avvia_vision_completo.sh"
echo ""
"@

# Salva script temporaneo
$setupScript | Out-File -Encoding UTF8 -FilePath "$env:TEMP\setup_vision.sh"

# Trasferisci e esegui
scp "$env:TEMP\setup_vision.sh" "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:/tmp/setup_vision.sh" 2>&1
ssh "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "bash /tmp/setup_vision.sh && rm /tmp/setup_vision.sh"

Write-Host ""
Write-Host "[5/5] Verifica installazione..." -ForegroundColor Yellow
ssh "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "cd $AI_ACCELERATOR_PATH && python3 -c 'from ultralytics import YOLO; print(\"✅ YOLOv8 OK\")' 2>&1"

Write-Host ""
Write-Host "==================================================================================" -ForegroundColor Green
Write-Host "✅ DEPLOY COMPLETATO AUTOMATICAMENTE!" -ForegroundColor Green
Write-Host "==================================================================================" -ForegroundColor Green
Write-Host ""
Write-Host "Sistema pronto su AI Accelerator!" -ForegroundColor Yellow
Write-Host ""
Write-Host "Per avviare:" -ForegroundColor Cyan
Write-Host "  ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor White
Write-Host "  cd ~/MekoAiAccelerator" -ForegroundColor White
Write-Host "  ./avvia_vision_completo.sh" -ForegroundColor White
Write-Host ""
Write-Host "Accesso browser:" -ForegroundColor Cyan
Write-Host "  http://$AI_ACCELERATOR_IP:8080" -ForegroundColor White
Write-Host ""




