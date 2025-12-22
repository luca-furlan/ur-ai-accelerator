# Script PowerShell per allineare file essenziali sul server AI Accelerator
# Esegui: .\ALLINEA_SERVER.ps1

$SERVER_IP = "192.168.10.191"
$SSH_USER = "lab"
$REMOTE_DIR = "~/MekoAiAccelerator"

Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "ALLINEAMENTO FILE ESSENZIALI SUL SERVER" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""

$filesToTransfer = @(
    "start_web_interface.py",
    "switch_controller.py",
    "attiva_controller.py",
    "attiva_controller.sh",
    "README.md",
    "remote_ur_control/web_interface.py"
)

Write-Host "Trasferimento file essenziali..." -ForegroundColor Yellow
Write-Host ""

$transferred = 0
foreach ($file in $filesToTransfer) {
    if (Test-Path $file) {
        Write-Host "  [TRANSFER] $file..." -ForegroundColor Gray
        
        if ($file -like "remote_ur_control/*") {
            $remotePath = "$REMOTE_DIR/remote_ur_control/web_interface.py"
            scp $file "${SSH_USER}@${SERVER_IP}:$remotePath" 2>&1 | Out-Null
        } else {
            scp $file "${SSH_USER}@${SERVER_IP}:$REMOTE_DIR/" 2>&1 | Out-Null
        }
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host "    OK: $file trasferito" -ForegroundColor Green
            $transferred++
        } else {
            Write-Host "    ERR: $file trasferimento fallito" -ForegroundColor Red
        }
    } else {
        Write-Host "  SKIP: $file non trovato localmente" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "VERIFICA FILE SUL SERVER" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""

$verification = ssh "${SSH_USER}@${SERVER_IP}" "cd $REMOTE_DIR && for f in start_web_interface.py switch_controller.py attiva_controller.py attiva_controller.sh README.md; do test -f \"\$f\" && echo \"OK: \$f\" || echo \"MISSING: \$f\"; done"
Write-Host $verification

Write-Host ""
Write-Host "Verifica endpoint switch_controller..." -ForegroundColor Yellow
$endpointCheck = ssh "${SSH_USER}@${SERVER_IP}" "cd $REMOTE_DIR && grep -c 'switch_controller.py robusto' remote_ur_control/web_interface.py 2>/dev/null || echo '0'"
if ($endpointCheck -gt 0) {
    Write-Host "  OK: Endpoint aggiornato" -ForegroundColor Green
} else {
    Write-Host "  WARN: Endpoint potrebbe non essere aggiornato" -ForegroundColor Yellow
}

Write-Host ""
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "ALLINEAMENTO COMPLETATO!" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""
Write-Host "File trasferiti: $transferred" -ForegroundColor Green
Write-Host ""
Write-Host "Prossimi passi:" -ForegroundColor Yellow
Write-Host "  1. Riavvia web interface sul server se è attiva"
Write-Host "  2. Verifica che l'endpoint switch_controller funzioni senza timeout"
Write-Host ""


