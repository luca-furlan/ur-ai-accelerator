# PowerShell script per copiare file su AI Accelerator
# Esegui: .\COPIA_SU_AI_ACCELERATOR.ps1

$AI_ACCELERATOR_IP = "192.168.10.191"
$SSH_USER = "lab"
$REMOTE_DIR = "~/MekoAiAccelerator"

Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "COPIA FILE SU AI ACCELERATOR" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""
Write-Host "Target: $SSH_USER@$AI_ACCELERATOR_IP" -ForegroundColor Yellow
Write-Host "Path: $REMOTE_DIR" -ForegroundColor Yellow
Write-Host ""

# Verifica SSH disponibile
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "❌ SSH non disponibile. Installa OpenSSH o usa Git Bash." -ForegroundColor Red
    exit 1
}

Write-Host "[1/4] Test connessione SSH..." -ForegroundColor Yellow
$sshTest = ssh -o ConnectTimeout=5 "$SSH_USER@$AI_ACCELERATOR_IP" "echo OK" 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "⚠️  Errore connessione SSH. Verifica:" -ForegroundColor Yellow
    Write-Host "   - IP corretto: $AI_ACCELERATOR_IP" -ForegroundColor Yellow
    Write-Host "   - Utente corretto: $SSH_USER" -ForegroundColor Yellow
    Write-Host "   - SSH configurato correttamente" -ForegroundColor Yellow
    exit 1
}
Write-Host "✅ Connessione SSH OK" -ForegroundColor Green
Write-Host ""

# Crea directory remota
Write-Host "[2/4] Creazione directory remota..." -ForegroundColor Yellow
ssh "$SSH_USER@$AI_ACCELERATOR_IP" "mkdir -p $REMOTE_DIR/remote_ur_control" 2>&1 | Out-Null
Write-Host "✅ Directory create" -ForegroundColor Green
Write-Host ""

# Lista file da copiare
Write-Host "[3/4] Trasferimento file..." -ForegroundColor Yellow
Write-Host ""

$filesToTransfer = @(
    @{Local = "vision_yolo_detector.py"; Remote = "$REMOTE_DIR/vision_yolo_detector.py"},
    @{Local = "avvia_vision_system.sh"; Remote = "$REMOTE_DIR/avvia_vision_system.sh"},
    @{Local = "FERMA_TUTTO.sh"; Remote = "$REMOTE_DIR/FERMA_TUTTO.sh"},
    @{Local = "remote_ur_control/web_interface.py"; Remote = "$REMOTE_DIR/remote_ur_control/web_interface.py"},
    @{Local = "SISTEMA_VISION_COMPLETO.md"; Remote = "$REMOTE_DIR/SISTEMA_VISION_COMPLETO.md"},
    @{Local = "AGGIORNA_SU_AI_ACCELERATOR.md"; Remote = "$REMOTE_DIR/AGGIORNA_SU_AI_ACCELERATOR.md"}
)

$transferred = 0
$failed = 0

foreach ($file in $filesToTransfer) {
    $localFile = $file.Local
    $remoteFile = $file.Remote
    
    if (Test-Path $localFile) {
        Write-Host "  [TRANSFER] $localFile..." -ForegroundColor Gray
        
        scp $localFile "${SSH_USER}@${AI_ACCELERATOR_IP}:$remoteFile" 2>&1 | Out-Null
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host "    ✅ $localFile copiato" -ForegroundColor Green
            $transferred++
        } else {
            Write-Host "    ❌ Errore copia $localFile" -ForegroundColor Red
            $failed++
        }
    } else {
        Write-Host "  ⚠️  SKIP: $localFile non trovato localmente" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "RISULTATO TRASFERIMENTO" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""
Write-Host "  ✅ File copiati: $transferred" -ForegroundColor Green
if ($failed -gt 0) {
    Write-Host "  ❌ File falliti: $failed" -ForegroundColor Red
}
Write-Host ""

# Rendi eseguibili gli script
Write-Host "[4/4] Impostazione permessi esecuzione..." -ForegroundColor Yellow
ssh "$SSH_USER@$AI_ACCELERATOR_IP" "cd $REMOTE_DIR && chmod +x vision_yolo_detector.py avvia_vision_system.sh FERMA_TUTTO.sh 2>/dev/null || true" 2>&1 | Out-Null
Write-Host "✅ Permessi impostati" -ForegroundColor Green
Write-Host ""

# Verifica file sul server
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "VERIFICA FILE SUL SERVER" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""

$verification = ssh "${SSH_USER}@${AI_ACCELERATOR_IP}" "cd $REMOTE_DIR && for f in vision_yolo_detector.py avvia_vision_system.sh FERMA_TUTTO.sh; do test -f \"\$f\" && echo \"✅ \$f\" || echo \"❌ MISSING: \$f\"; done"
Write-Host $verification

$webInterfaceCheck = ssh "${SSH_USER}@${AI_ACCELERATOR_IP}" "test -f $REMOTE_DIR/remote_ur_control/web_interface.py && echo '✅ remote_ur_control/web_interface.py' || echo '❌ MISSING: remote_ur_control/web_interface.py'"
Write-Host $webInterfaceCheck

Write-Host ""
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "✅ COPIA COMPLETATA" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""
Write-Host "Ora puoi avviare la web interface sul server:" -ForegroundColor Yellow
Write-Host "  ssh $SSH_USER@$AI_ACCELERATOR_IP" -ForegroundColor White
Write-Host "  cd ~/MekoAiAccelerator" -ForegroundColor White
Write-Host "  bash avvia_web_interface.sh" -ForegroundColor White
Write-Host ""
