# PowerShell script per deploy fix switch_controller
# Copia i file modificati sulla macchina remota

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$AI_ACCELERATOR_PATH = "~/MekoAiAccelerator"

Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host "DEPLOY FIX SWITCH CONTROLLER" -ForegroundColor Cyan
Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Target: $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Yellow
Write-Host "Path: $AI_ACCELERATOR_PATH" -ForegroundColor Yellow
Write-Host ""

# Verifica SSH disponibile
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "❌ SSH non disponibile. Installa OpenSSH o usa Git Bash." -ForegroundColor Red
    exit 1
}

Write-Host "[1/3] Test connessione SSH..." -ForegroundColor Yellow
$sshTest = ssh -o ConnectTimeout=5 "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" "echo OK" 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "⚠️  Errore connessione SSH. Verifica:" -ForegroundColor Yellow
    Write-Host "   - IP corretto: $AI_ACCELERATOR_IP" -ForegroundColor Yellow
    Write-Host "   - Utente corretto: $AI_ACCELERATOR_USER" -ForegroundColor Yellow
    Write-Host "   - SSH configurato correttamente" -ForegroundColor Yellow
    exit 1
}
Write-Host "✅ Connessione SSH OK" -ForegroundColor Green
Write-Host ""

Write-Host "[2/3] Trasferimento file modificati..." -ForegroundColor Yellow

# Copia switch_controller.py
Write-Host "  - switch_controller.py..." -ForegroundColor Cyan
scp switch_controller.py "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/" 2>&1
if ($LASTEXITCODE -eq 0) {
    Write-Host "    ✅ switch_controller.py copiato" -ForegroundColor Green
} else {
    Write-Host "    ❌ Errore copia switch_controller.py" -ForegroundColor Red
    exit 1
}

# Copia web_interface.py
Write-Host "  - remote_ur_control/web_interface.py..." -ForegroundColor Cyan
scp remote_ur_control/web_interface.py "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/remote_ur_control/" 2>&1
if ($LASTEXITCODE -eq 0) {
    Write-Host "    ✅ web_interface.py copiato" -ForegroundColor Green
} else {
    Write-Host "    ❌ Errore copia web_interface.py" -ForegroundColor Red
    exit 1
}

# Copia ros2_bridge_fixed.py (potrebbe essere stato modificato)
Write-Host "  - ros2_bridge_fixed.py..." -ForegroundColor Cyan
scp ros2_bridge_fixed.py "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/" 2>&1 | Out-Null

Write-Host ""
Write-Host "[3/3] Riavvio web interface..." -ForegroundColor Yellow
Write-Host "  (Ferma processo esistente e riavvia)" -ForegroundColor Gray

# Ferma web interface esistente
ssh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "pkill -f 'web_interface' || true" 2>&1 | Out-Null
Start-Sleep -Seconds 2

# Copia script avvio
Write-Host "  - avvia_web_interface.sh..." -ForegroundColor Cyan
scp avvia_web_interface.sh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${AI_ACCELERATOR_PATH}/" 2>&1 | Out-Null
ssh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "chmod +x ${AI_ACCELERATOR_PATH}/avvia_web_interface.sh" 2>&1 | Out-Null

# Riavvia web interface usando lo script
Write-Host "  Avvio web interface in background..." -ForegroundColor Gray
ssh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "cd $AI_ACCELERATOR_PATH && nohup ./avvia_web_interface.sh > /tmp/web_interface.log 2>&1 &" 2>&1 | Out-Null

Start-Sleep -Seconds 3

# Verifica che sia avviata
$webPid = ssh "${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}" "pgrep -f 'web_interface' | head -1" 2>&1
if ($webPid -and $webPid -match '^\d+$') {
    Write-Host "  ✅ Web interface avviata (PID: $webPid)" -ForegroundColor Green
} else {
    Write-Host "  ⚠️  Web interface potrebbe non essere avviata. Verifica manualmente." -ForegroundColor Yellow
    Write-Host "     Log: ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP 'tail -50 /tmp/web_interface.log'" -ForegroundColor Gray
}

Write-Host ""
Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host "✅ DEPLOY COMPLETATO" -ForegroundColor Green
Write-Host "==================================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "File copiati:" -ForegroundColor Yellow
Write-Host "  - switch_controller.py" -ForegroundColor White
Write-Host "  - remote_ur_control/web_interface.py" -ForegroundColor White
Write-Host ""
Write-Host "Web interface:" -ForegroundColor Yellow
Write-Host "  http://$AI_ACCELERATOR_IP:8080" -ForegroundColor White
Write-Host ""
Write-Host "Per vedere i log:" -ForegroundColor Yellow
Write-Host "  ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP 'tail -f /tmp/web_interface.log'" -ForegroundColor Gray
Write-Host ""

