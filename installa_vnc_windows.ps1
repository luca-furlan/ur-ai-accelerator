# ================================================================================
# INSTALLAZIONE VNC CLIENT SU WINDOWS
# ================================================================================
# Questo script installa TightVNC Viewer per connettersi al robot
# ================================================================================

Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host "INSTALLAZIONE VNC CLIENT PER CONNESSIONE AL ROBOT" -ForegroundColor Cyan
Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host ""

# Informazioni connessione
$ROBOT_IP = "192.168.10.191"
$VNC_PORT_1 = "5901"
$VNC_PORT_2 = "5902"
$VNC_PASSWORD = "easybot"

Write-Host "[*] Informazioni connessione:" -ForegroundColor Yellow
Write-Host "   IP Robot: $ROBOT_IP" -ForegroundColor White
Write-Host "   Porta VNC: $VNC_PORT_1 o $VNC_PORT_2" -ForegroundColor White
Write-Host "   Password: $VNC_PASSWORD" -ForegroundColor White
Write-Host ""

# Verifica se TightVNC e' gia' installato
$tightVNCInstalled = Get-Command "tvnviewer.exe" -ErrorAction SilentlyContinue

if ($tightVNCInstalled) {
    Write-Host "[OK] TightVNC Viewer e' gia' installato!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Avvio TightVNC Viewer..." -ForegroundColor Yellow
    Start-Process "tvnviewer.exe"
    Write-Host ""
    Write-Host "[*] Istruzioni:" -ForegroundColor Cyan
    Write-Host "   1. Inserisci: $ROBOT_IP`:$VNC_PORT_1" -ForegroundColor White
    Write-Host "   2. Se non funziona, prova: $ROBOT_IP`:$VNC_PORT_2" -ForegroundColor White
    Write-Host "   3. Password: $VNC_PASSWORD" -ForegroundColor White
    exit 0
}

Write-Host "[*] Download TightVNC Viewer..." -ForegroundColor Yellow

# Crea cartella temporanea
$tempDir = "$env:TEMP\vnc_install"
if (-not (Test-Path $tempDir)) {
    New-Item -ItemType Directory -Path $tempDir -Force | Out-Null
}

# URL download TightVNC Viewer (versione portabile, piu' semplice)
$vncUrl = "https://www.tightvnc.com/download/2.8.8/tightvnc-2.8.8-gpl-setup-64bit.msi"
$installerPath = "$tempDir\tightvnc-viewer.msi"

try {
    Write-Host "   Download da: $vncUrl" -ForegroundColor Gray
    
    # Download con progress bar
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri $vncUrl -OutFile $installerPath -UseBasicParsing
    
    if (Test-Path $installerPath) {
        Write-Host "   [OK] Download completato" -ForegroundColor Green
    } else {
        throw "File non scaricato correttamente"
    }
} catch {
    Write-Host "   [ERRORE] Errore nel download: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "[*] Alternativa: Scarica manualmente da:" -ForegroundColor Yellow
    Write-Host "   https://www.tightvnc.com/download.php" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "   Oppure usa RealVNC Viewer:" -ForegroundColor Yellow
    Write-Host "   https://www.realvnc.com/en/connect/download/viewer/" -ForegroundColor Cyan
    exit 1
}

Write-Host ""
Write-Host "[*] Installazione TightVNC Viewer..." -ForegroundColor Yellow

try {
    # Installa in modalita' silenziosa (solo viewer, non server)
    $installArgs = "/i `"$installerPath`" /quiet /norestart ADDLOCAL=Viewer"
    Start-Process msiexec.exe -ArgumentList $installArgs -Wait -NoNewWindow
    
    Write-Host "   [OK] Installazione completata" -ForegroundColor Green
} catch {
    Write-Host "   [ERRORE] Errore durante l'installazione: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "[*] Prova a installare manualmente:" -ForegroundColor Yellow
    Write-Host "   1. Apri: $installerPath" -ForegroundColor White
    Write-Host "   2. Installa solo 'TightVNC Viewer' (non il server)" -ForegroundColor White
    exit 1
}

Write-Host ""
Write-Host "[*] Verifica installazione..." -ForegroundColor Yellow

# Attendi un momento per il completamento
Start-Sleep -Seconds 2

# Cerca TightVNC Viewer nei percorsi comuni
$vncPaths = @(
    "${env:ProgramFiles}\TightVNC\tvnviewer.exe",
    "${env:ProgramFiles(x86)}\TightVNC\tvnviewer.exe",
    "$env:LOCALAPPDATA\Programs\TightVNC\tvnviewer.exe"
)

$vncFound = $false
foreach ($path in $vncPaths) {
    if (Test-Path $path) {
        Write-Host "   [OK] TightVNC trovato: $path" -ForegroundColor Green
        $vncFound = $true
        
        # Avvia TightVNC Viewer
        Write-Host ""
        Write-Host "[*] Avvio TightVNC Viewer..." -ForegroundColor Yellow
        Start-Process $path
        
        break
    }
}

if (-not $vncFound) {
    Write-Host "   [ATTENZIONE] TightVNC non trovato nei percorsi standard" -ForegroundColor Yellow
    Write-Host "   Cerca 'TightVNC Viewer' nel menu Start" -ForegroundColor White
}

Write-Host ""
Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host "[OK] INSTALLAZIONE COMPLETATA" -ForegroundColor Green
Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "[*] ISTRUZIONI CONNESSIONE:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. Nella finestra TightVNC Viewer che si e' aperta:" -ForegroundColor White
Write-Host "   - Inserisci: $ROBOT_IP`:$VNC_PORT_1" -ForegroundColor Cyan
Write-Host "   - Clicca 'Connect'" -ForegroundColor White
Write-Host ""
Write-Host "2. Se la porta $VNC_PORT_1 non funziona, prova:" -ForegroundColor White
Write-Host "   - $ROBOT_IP`:$VNC_PORT_2" -ForegroundColor Cyan
Write-Host ""
Write-Host "3. Quando richiesto, inserisci la password:" -ForegroundColor White
Write-Host "   Password: $VNC_PASSWORD" -ForegroundColor Cyan
Write-Host ""
Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host ""

# Verifica connessione di rete al robot
Write-Host "[*] Verifica connessione al robot..." -ForegroundColor Yellow
$pingResult = Test-Connection -ComputerName $ROBOT_IP -Count 2 -Quiet -ErrorAction SilentlyContinue

if ($pingResult) {
    Write-Host "   [OK] Robot raggiungibile ($ROBOT_IP)" -ForegroundColor Green
} else {
    Write-Host "   [ATTENZIONE] Robot non raggiungibile ($ROBOT_IP)" -ForegroundColor Yellow
    Write-Host "   Verifica la connessione di rete" -ForegroundColor White
}

Write-Host ""
Write-Host "[*] Se hai problemi:" -ForegroundColor Yellow
Write-Host "   - Verifica che il robot sia acceso" -ForegroundColor White
Write-Host "   - Verifica che VNC sia avviato sul robot" -ForegroundColor White
Write-Host "   - Controlla il firewall Windows" -ForegroundColor White
Write-Host ""

# Pulisci file temporanei
if (Test-Path $tempDir) {
    Remove-Item $tempDir -Recurse -Force -ErrorAction SilentlyContinue
}
