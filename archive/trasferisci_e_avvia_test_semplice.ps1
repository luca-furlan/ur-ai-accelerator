# Script SEMPLICE - Trasferisce file e avvia test automaticamente
# Usa SSH con password (richiede interazione o configurazione SSH key)

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$REMOTE_DIR = "~/MekoAiAccelerator"

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "TRASFERIMENTO E AVVIO TEST" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Verifica connessione
Write-Host "Verifica connessione..." -ForegroundColor Yellow
$ping = Test-Connection -ComputerName $AI_ACCELERATOR_IP -Count 1 -Quiet
if (-not $ping) {
    Write-Host "❌ AI Accelerator NON raggiungibile!" -ForegroundColor Red
    exit 1
}
Write-Host "✅ AI Accelerator raggiungibile" -ForegroundColor Green
Write-Host ""

# Verifica SSH
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "❌ SSH non disponibile!" -ForegroundColor Red
    Write-Host "Installa OpenSSH: Add-WindowsCapability -Online -Name OpenSSH.Client" -ForegroundColor Yellow
    exit 1
}

Write-Host "Trasferimento file..." -ForegroundColor Yellow
Write-Host ""

# Crea script bash temporaneo per trasferimento
$transferScript = @"
#!/bin/bash
cd `$HOME/MekoAiAccelerator
mkdir -p test

# Crea file test_sistema_completo.py
cat > test_sistema_completo.py << 'ENDOFFILE'
$(Get-Content "test_sistema_completo.py" -Raw)
ENDOFFILE

# Crea file quick_check_sistema.py
cat > quick_check_sistema.py << 'ENDOFFILE'
$(Get-Content "quick_check_sistema.py" -Raw)
ENDOFFILE

# Crea directory test e file
mkdir -p test
$(Get-ChildItem "test\*.py" | ForEach-Object {
    $fileName = $_.Name
    $fileContent = Get-Content $_.FullName -Raw
    @"
cat > test/$fileName << 'ENDOFFILE'
$fileContent
ENDOFFILE
"@
})

chmod +x test_sistema_completo.py quick_check_sistema.py test/*.py
echo "✅ File creati e resi eseguibili"
"@

# Salva script temporaneo
$tempScript = [System.IO.Path]::GetTempFileName() + ".sh"
$transferScript | Out-File -FilePath $tempScript -Encoding UTF8

Write-Host "   📤 Trasferisco script di setup..." -ForegroundColor Gray

# Trasferisci script
$scpCmd = "scp -o StrictHostKeyChecking=no `"$tempScript`" $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP`:~/setup_test.sh"
Invoke-Expression $scpCmd 2>&1 | Out-Null

# Esegui script su remoto
Write-Host "   🔧 Eseguo setup su remoto..." -ForegroundColor Gray
$sshCmd = "ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP `"bash ~/setup_test.sh && rm ~/setup_test.sh`""
Invoke-Expression $sshCmd

# Rimuovi script temporaneo
Remove-Item $tempScript -ErrorAction SilentlyContinue

Write-Host "   ✅ File trasferiti!" -ForegroundColor Green
Write-Host ""

# Avvia test
Write-Host "Avvio quick check..." -ForegroundColor Yellow
Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan

$testCmd = "ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP `"cd $REMOTE_DIR && python3 quick_check_sistema.py`""
Invoke-Expression $testCmd

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "✅ Completato!" -ForegroundColor Green











