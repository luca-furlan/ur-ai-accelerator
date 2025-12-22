# Script PowerShell per scaricare URCap e prepararlo per USB

$URCAP_VERSION = "1.0.5"
$URCAP_FILE = "externalcontrol-$URCAP_VERSION.urcap"
$URCAP_URL = "https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v$URCAP_VERSION/$URCAP_FILE"

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "PREPARAZIONE URCAP PER USB" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# 1. Scarica URCap
Write-Host "1. Download External Control URCap..." -ForegroundColor Yellow
if (-not (Test-Path $URCAP_FILE)) {
    Write-Host "   Download da GitHub..." -ForegroundColor Gray
    try {
        Invoke-WebRequest -Uri $URCAP_URL -OutFile $URCAP_FILE -UseBasicParsing
        Write-Host "   ✅ URCap scaricato" -ForegroundColor Green
    } catch {
        Write-Host "   ❌ Download fallito!" -ForegroundColor Red
        Write-Host "   URL: $URCAP_URL" -ForegroundColor Yellow
        exit 1
    }
} else {
    Write-Host "   ✅ URCap già presente" -ForegroundColor Green
}

# Verifica file
if (-not (Test-Path $URCAP_FILE)) {
    Write-Host "   ❌ File non trovato!" -ForegroundColor Red
    exit 1
}

$fileInfo = Get-Item $URCAP_FILE
Write-Host "   Dimensione: $($fileInfo.Length) bytes" -ForegroundColor Gray

# 2. Istruzioni
Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "✅ FILE PRONTO PER USB!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "PROSSIMI PASSI:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. Copia questo file su una chiavetta USB:" -ForegroundColor White
Write-Host "   $($fileInfo.FullName)" -ForegroundColor Cyan
Write-Host ""
Write-Host "2. Inserisci la USB nel Teach Pendant" -ForegroundColor White
Write-Host ""
Write-Host "3. Sul Teach Pendant:" -ForegroundColor White
Write-Host "   - Vai su: ☰ Menu → Settings → System → URCaps" -ForegroundColor Cyan
Write-Host "   - Clicca '+' (aggiungi)" -ForegroundColor Cyan
Write-Host "   - Naviga alla USB" -ForegroundColor Cyan
Write-Host "   - Seleziona: $URCAP_FILE" -ForegroundColor Cyan
Write-Host "   - Riavvia il robot quando richiesto" -ForegroundColor Cyan
Write-Host ""
Write-Host "4. Dopo il riavvio:" -ForegroundColor White
Write-Host "   - Vai su: Program → New Program" -ForegroundColor Cyan
Write-Host "   - Nel menu URCaps, trova 'External Control'" -ForegroundColor Cyan
Write-Host "   - Trascina il nodo nel programma" -ForegroundColor Cyan
Write-Host "   - Configura:" -ForegroundColor Cyan
Write-Host "     * Host IP: 192.168.10.191" -ForegroundColor White
Write-Host "     * Port: 50002" -ForegroundColor White
Write-Host "   - Salva il programma" -ForegroundColor Cyan
Write-Host "   - Premi PLAY" -ForegroundColor Cyan
Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan











