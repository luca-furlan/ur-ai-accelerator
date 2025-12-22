# PowerShell script per deploy automatico - VERSIONE CORRETTA
$ErrorActionPreference = "Continue"

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$AI_ACCELERATOR_PATH = "~/MekoAiAccelerator"

Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host "AUTOMATIC DEPLOY TO AI ACCELERATOR" -ForegroundColor Cyan
Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Target: $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Yellow
Write-Host ""

# Verifica SSH
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "❌ SSH non disponibile" -ForegroundColor Red
    exit 1
}

Write-Host "[1/4] Trasferimento file..." -ForegroundColor Yellow
scp vision_yolo_detector.py "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/" 2>&1 | Out-Null
scp vision_robot_coordinator.py "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/" 2>&1 | Out-Null
scp remote_ur_control/vision_web_api.py "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/remote_ur_control/" 2>&1 | Out-Null
Write-Host "✅ File trasferiti" -ForegroundColor Green

Write-Host "[2/4] Installazione dipendenze Python..." -ForegroundColor Yellow
ssh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "cd ${AI_ACCELERATOR_PATH} && pip3 install ultralytics opencv-python numpy flask requests --user -q 2>&1 | grep -v 'already satisfied' || true"
Write-Host "✅ Python packages installati" -ForegroundColor Green

Write-Host "[3/4] Installazione ROS2 packages..." -ForegroundColor Yellow
Write-Host "  (Potrebbe richiedere password sudo)" -ForegroundColor Yellow
ssh -t "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "sudo apt install -y ros-humble-cv-bridge ros-humble-vision-msgs -qq 2>&1 | grep -v 'already' || true"
Write-Host "✅ ROS2 packages installati" -ForegroundColor Green

Write-Host "[4/4] Setup e integrazione..." -ForegroundColor Yellow

# Script bash con line endings corretti
$bashScript = @'
#!/bin/bash
set -e
cd ~/MekoAiAccelerator

chmod +x *.py 2>/dev/null || true
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

# Integra Vision API
if ! grep -q "vision_web_api" remote_ur_control/web_interface.py 2>/dev/null; then
    cp remote_ur_control/web_interface.py remote_ur_control/web_interface.py.backup 2>/dev/null || true
    
    # Aggiungi import
    python3 << 'PYEOF'
import re

with open('remote_ur_control/web_interface.py', 'r') as f:
    content = f.read()

# Aggiungi import dopo "from flask import"
if 'vision_web_api' not in content:
    import_pattern = r'(from flask import[^\n]+)'
    vision_import = '''from flask import\\1

# Vision API
try:
    from .vision_web_api import add_vision_routes_to_app
    VISION_API_AVAILABLE = True
except ImportError:
    VISION_API_AVAILABLE = False
    print("⚠️ Vision API non disponibile")'''
    
    content = re.sub(import_pattern, vision_import, content, count=1)
    
    # Aggiungi inizializzazione dopo "app = Flask"
    app_pattern = r'(app = Flask\([^\n]+)'
    vision_init = '''\\1

# Initialize Vision API
if VISION_API_AVAILABLE:
    try:
        vision_api = add_vision_routes_to_app(app)
        print("✅ Vision API integrata")
    except Exception as e:
        print(f"⚠️ Errore integrazione Vision API: {e}")'''
    
    content = re.sub(app_pattern, vision_init, content, count=1)
    
    with open('remote_ur_control/web_interface.py', 'w') as f:
        f.write(content)
    print("✅ Vision API integrata")
else:
    print("✅ Vision API già integrata")
PYEOF
fi

# Crea script avvio
cat > avvia_vision_completo.sh << 'EOFBASH'
#!/bin/bash
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
export PYTHONPATH=$(pwd):$PYTHONPATH
export UR_ROBOT_IP=${UR_ROBOT_IP:-192.168.10.194}

if ros2 pkg list | grep -q orbbec_camera; then
    ros2 launch orbbec_camera gemini_330_series.launch.py > /tmp/camera.log 2>&1 &
    sleep 3
fi

python3 vision_yolo_detector.py > /tmp/vision.log 2>&1 &
sleep 2

python3 vision_robot_coordinator.py > /tmp/coordinator.log 2>&1 &
sleep 1

echo "✅ Vision system avviato!"
python3 -m remote_ur_control.web_interface
EOFBASH

chmod +x avvia_vision_completo.sh
echo "✅ Script avvio creato"
'@

# Salva script temporaneo con encoding UTF8 senza BOM
$bashScript | Out-File -Encoding ASCII -FilePath "$env:TEMP\setup_vision.sh" -NoNewline
$bashScript | Out-File -Encoding ASCII -FilePath "$env:TEMP\setup_vision.sh"

# Trasferisci e esegui
scp "$env:TEMP\setup_vision.sh" "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:/tmp/setup_vision.sh" 2>&1 | Out-Null
ssh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "bash /tmp/setup_vision.sh && rm /tmp/setup_vision.sh"

Write-Host "✅ Setup completato" -ForegroundColor Green

Write-Host ""
Write-Host "==================================================================================" -ForegroundColor Green
Write-Host "✅ DEPLOY COMPLETATO!" -ForegroundColor Green
Write-Host "==================================================================================" -ForegroundColor Green
Write-Host ""
Write-Host "Per avviare:" -ForegroundColor Cyan
Write-Host "  ssh ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" -ForegroundColor White
Write-Host "  cd ~/MekoAiAccelerator" -ForegroundColor White
Write-Host "  ./avvia_vision_completo.sh" -ForegroundColor White
Write-Host ""
Write-Host "Accesso browser: http://${AI_ACCELERATOR_IP}:8080" -ForegroundColor Cyan
Write-Host ""




