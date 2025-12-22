# Script RAPIDO - Trasferisce e avvia test SUBITO

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$REMOTE_DIR = "~/MekoAiAccelerator"

Write-Host "TRASFERIMENTO E AVVIO TEST AUTOMATICO" -ForegroundColor Cyan
Write-Host ""

# Verifica SSH
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "ERRORE: SSH non disponibile!" -ForegroundColor Red
    Write-Host "Installa: Add-WindowsCapability -Online -Name OpenSSH.Client" -ForegroundColor Yellow
    exit 1
}

# Verifica connessione
Write-Host "Verifica connessione..." -ForegroundColor Yellow
$ping = Test-Connection -ComputerName $AI_ACCELERATOR_IP -Count 1 -Quiet
if (-not $ping) {
    Write-Host "ERRORE: AI Accelerator NON raggiungibile!" -ForegroundColor Red
    exit 1
}
Write-Host "OK: AI Accelerator raggiungibile" -ForegroundColor Green
Write-Host ""

# Verifica SCP
if (-not (Get-Command scp -ErrorAction SilentlyContinue)) {
    Write-Host "ERRORE: SCP non disponibile!" -ForegroundColor Red
    Write-Host "Installa OpenSSH Client" -ForegroundColor Yellow
    exit 1
}

Write-Host "Trasferimento file..." -ForegroundColor Yellow

# Crea directory remota
$mkdirCmd = "ssh -o StrictHostKeyChecking=no ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP} mkdir -p ${REMOTE_DIR}/test"
Invoke-Expression $mkdirCmd 2>&1 | Out-Null

# Trasferisci file
if (Test-Path "test_sistema_completo.py") {
    Write-Host "   Trasferisco: test_sistema_completo.py" -ForegroundColor Gray
    $scp1 = "scp -o StrictHostKeyChecking=no test_sistema_completo.py ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${REMOTE_DIR}/"
    Invoke-Expression $scp1 2>&1 | Out-Null
    Write-Host "      OK: Trasferito" -ForegroundColor Green
}

if (Test-Path "quick_check_sistema.py") {
    Write-Host "   Trasferisco: quick_check_sistema.py" -ForegroundColor Gray
    $scp2 = "scp -o StrictHostKeyChecking=no quick_check_sistema.py ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${REMOTE_DIR}/"
    Invoke-Expression $scp2 2>&1 | Out-Null
    Write-Host "      OK: Trasferito" -ForegroundColor Green
}

if (Test-Path "test") {
    Write-Host "   Trasferisco: directory test" -ForegroundColor Gray
    $scp3 = "scp -o StrictHostKeyChecking=no -r test ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP}:${REMOTE_DIR}/"
    Invoke-Expression $scp3 2>&1 | Out-Null
    Write-Host "      OK: Trasferita" -ForegroundColor Green
}

# Rendi eseguibili
Write-Host "   Rendo eseguibili i file..." -ForegroundColor Gray
$chmod = "ssh -o StrictHostKeyChecking=no ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP} 'cd ${REMOTE_DIR}; chmod +x test_sistema_completo.py quick_check_sistema.py test/*.py'"
Invoke-Expression $chmod 2>&1 | Out-Null

Write-Host "   OK: File trasferiti!" -ForegroundColor Green
Write-Host ""

# Avvia test
Write-Host "AVVIO QUICK CHECK..." -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan

$test = "ssh -o StrictHostKeyChecking=no ${AI_ACCELERATOR_USER}@${AI_ACCELERATOR_IP} 'cd ${REMOTE_DIR}; python3 quick_check_sistema.py'"
Invoke-Expression $test

Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "Test completato!" -ForegroundColor Green
