# Script PowerShell per trasferire e avviare test automaticamente

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$AI_ACCELERATOR_PASS = "easybot"
$REMOTE_DIR = "~/MekoAiAccelerator"
$LOCAL_DIR = $PSScriptRoot

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "TRASFERIMENTO E AVVIO TEST AUTOMATICO" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Verifica connessione
Write-Host "1. Verifica connessione..." -ForegroundColor Yellow
$ping = Test-Connection -ComputerName $AI_ACCELERATOR_IP -Count 1 -Quiet
if (-not $ping) {
    Write-Host "❌ AI Accelerator NON raggiungibile!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ AI Accelerator raggiungibile" -ForegroundColor Green
Write-Host ""

# Verifica se SCP è disponibile
Write-Host "2. Verifica strumenti SSH..." -ForegroundColor Yellow
$hasSCP = Get-Command scp -ErrorAction SilentlyContinue
$hasSSH = Get-Command ssh -ErrorAction SilentlyContinue

if (-not $hasSCP -and -not $hasSSH) {
    Write-Host "❌ SCP/SSH non disponibile!" -ForegroundColor Red
    Write-Host ""
    Write-Host "Installa OpenSSH:" -ForegroundColor Yellow
    Write-Host "  Add-WindowsCapability -Online -Name OpenSSH.Client" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Oppure usa WinSCP/FileZilla per trasferire manualmente:" -ForegroundColor Yellow
    Write-Host "  Host: $AI_ACCELERATOR_IP" -ForegroundColor Cyan
    Write-Host "  User: $AI_ACCELERATOR_USER" -ForegroundColor Cyan
    Write-Host "  Password: $AI_ACCELERATOR_PASS" -ForegroundColor Cyan
    exit 1
}

Write-Host "✅ Strumenti SSH disponibili" -ForegroundColor Green
Write-Host ""

# File da trasferire
$filesToTransfer = @(
    "test_sistema_completo.py",
    "quick_check_sistema.py"
)

$testFiles = @(
    "test\run_all_tests.py",
    "test\test_connettivita_robot.py",
    "test\test_ros2_driver.py",
    "test\test_camera_orbbec.py",
    "test\test_mujoco.py",
    "test\test_ai_components.py",
    "test\test_web_interface.py",
    "test\__init__.py",
    "test\README.md"
)

Write-Host "3. Trasferimento file..." -ForegroundColor Yellow

# Crea directory test su remoto
Write-Host "   Creazione directory remota..." -ForegroundColor Gray
if ($hasSSH) {
    $createDirCmd = "ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP `"mkdir -p $REMOTE_DIR/test`""
    Invoke-Expression $createDirCmd 2>&1 | Out-Null
} else {
    $createDirCmd = "scp -o StrictHostKeyChecking=no -r test $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/ 2>&1"
    # Fallback: crea directory con primo file
}

# Trasferisci file principali
foreach ($file in $filesToTransfer) {
    $localPath = Join-Path $LOCAL_DIR $file
    if (Test-Path $localPath) {
        Write-Host "   📤 Trasferisco: $file" -ForegroundColor Gray
        if ($hasSCP) {
            $scpCmd = "scp -o StrictHostKeyChecking=no `"$localPath`" $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:$REMOTE_DIR/$file"
            $result = Invoke-Expression $scpCmd 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "      ✅ $file trasferito" -ForegroundColor Green
            } else {
                Write-Host "      ❌ Errore trasferimento $file" -ForegroundColor Red
                Write-Host "      $result" -ForegroundColor Red
            }
        } else {
            # Usa SSH con here-document
            $content = Get-Content $localPath -Raw
            $sshCmd = @"
ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "cat > $REMOTE_DIR/$file" << 'EOF'
$content
EOF
"@
            Invoke-Expression $sshCmd 2>&1 | Out-Null
        }
    } else {
        Write-Host "   ⚠️  File non trovato: $file" -ForegroundColor Yellow
    }
}

# Trasferisci directory test
Write-Host "   📤 Trasferisco directory test..." -ForegroundColor Gray
if ($hasSCP) {
    $testDir = Join-Path $LOCAL_DIR "test"
    if (Test-Path $testDir) {
        $scpCmd = "scp -o StrictHostKeyChecking=no -r `"$testDir`" $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:$REMOTE_DIR/"
        $result = Invoke-Expression $scpCmd 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "      ✅ Directory test trasferita" -ForegroundColor Green
        } else {
            Write-Host "      ❌ Errore trasferimento directory test" -ForegroundColor Red
        }
    }
} else {
    # Trasferisci file per file
    foreach ($file in $testFiles) {
        $localPath = Join-Path $LOCAL_DIR $file
        if (Test-Path $localPath) {
            $remoteFile = $file -replace "\\", "/"
            Write-Host "      📤 $remoteFile" -ForegroundColor Gray
            $content = Get-Content $localPath -Raw
            $sshCmd = @"
ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "mkdir -p $REMOTE_DIR/test && cat > $REMOTE_DIR/$remoteFile" << 'EOF'
$content
EOF
"@
            Invoke-Expression $sshCmd 2>&1 | Out-Null
        }
    }
}

Write-Host "   ✅ File trasferiti" -ForegroundColor Green
Write-Host ""

# Rendi eseguibili
Write-Host "4. Rendo eseguibili i file..." -ForegroundColor Yellow
$chmodCmd = "ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP `"cd $REMOTE_DIR && chmod +x test_sistema_completo.py quick_check_sistema.py test/*.py 2>/dev/null; echo 'OK'`""
Invoke-Expression $chmodCmd 2>&1 | Out-Null
Write-Host "   ✅ File resi eseguibili" -ForegroundColor Green
Write-Host ""

# Chiedi quale test avviare
Write-Host "5. Quale test vuoi avviare?" -ForegroundColor Yellow
Write-Host "   1) Quick check (veloce)" -ForegroundColor Cyan
Write-Host "   2) Verifica completa" -ForegroundColor Cyan
Write-Host "   3) Tutti i test funzionali" -ForegroundColor Cyan
Write-Host "   4) Test connettività robot" -ForegroundColor Cyan
Write-Host "   5) Test ROS2 driver" -ForegroundColor Cyan
Write-Host "   6) Test camera Orbbec" -ForegroundColor Cyan
Write-Host "   7) Test MuJoCo" -ForegroundColor Cyan
Write-Host "   8) Test componenti AI" -ForegroundColor Cyan
Write-Host "   9) Test web interface" -ForegroundColor Cyan
Write-Host "   0) Nessuno (solo trasferimento)" -ForegroundColor Cyan
Write-Host ""

$choice = Read-Host "Scelta (default: 1)"

$testCommand = switch ($choice) {
    "1" { "python3 quick_check_sistema.py" }
    "2" { "python3 test_sistema_completo.py" }
    "3" { "python3 test/run_all_tests.py" }
    "4" { "python3 test/test_connettivita_robot.py" }
    "5" { "python3 test/test_ros2_driver.py" }
    "6" { "python3 test/test_camera_orbbec.py" }
    "7" { "python3 test/test_mujoco.py" }
    "8" { "python3 test/test_ai_components.py" }
    "9" { "python3 test/test_web_interface.py" }
    default { "python3 quick_check_sistema.py" }
}

if ($choice -ne "0") {
    Write-Host ""
    Write-Host "6. Avvio test..." -ForegroundColor Yellow
    Write-Host "   Comando: $testCommand" -ForegroundColor Gray
    Write-Host ""
    Write-Host "==========================================" -ForegroundColor Cyan
    Write-Host ""
    
    $runCmd = "ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP `"cd $REMOTE_DIR && $testCommand`""
    Invoke-Expression $runCmd
    
    Write-Host ""
    Write-Host "==========================================" -ForegroundColor Cyan
    Write-Host "✅ Test completato!" -ForegroundColor Green
    Write-Host "==========================================" -ForegroundColor Cyan
} else {
    Write-Host ""
    Write-Host "✅ File trasferiti. Avvia manualmente quando vuoi:" -ForegroundColor Green
    Write-Host "   ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Cyan
    Write-Host "   cd $REMOTE_DIR" -ForegroundColor Cyan
    Write-Host "   python3 quick_check_sistema.py" -ForegroundColor Cyan
}











