# Script PowerShell per verificare TightVNC e connessione robot
# Esegui: .\verifica_tightvnc_e_robot.ps1

$ROBOT_IP = "192.168.10.194"
$VNC_PORT = 5900

Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "VERIFICA TIGHTVNC E CONNESSIONE ROBOT" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host ""

# ========================================================================
# 1. VERIFICA TIGHTVNC INSTALLATO
# ========================================================================
Write-Host "[1/3] Verifica TightVNC Viewer..." -ForegroundColor Yellow

$vncPaths = @(
    "${env:ProgramFiles}\TightVNC\tvnviewer.exe",
    "${env:ProgramFiles(x86)}\TightVNC\tvnviewer.exe",
    "${env:LOCALAPPDATA}\Programs\TightVNC Viewer\tvnviewer.exe",
    "${env:ProgramFiles}\TightVNC Viewer\tvnviewer.exe",
    "${env:ProgramFiles(x86)}\TightVNC Viewer\tvnviewer.exe"
)

$vncFound = $false
$vncPath = $null

foreach ($path in $vncPaths) {
    if (Test-Path $path) {
        $vncFound = $true
        $vncPath = $path
        Write-Host "  [OK] TightVNC Viewer trovato: $path" -ForegroundColor Green
        break
    }
}

if (-not $vncFound) {
    Write-Host "  [WARN] TightVNC Viewer NON trovato nei percorsi standard" -ForegroundColor Yellow
    
    # Cerca in tutto il sistema
    Write-Host "  [INFO] Ricerca TightVNC in tutto il sistema..." -ForegroundColor Cyan
    $searchResult = Get-Command "tvnviewer" -ErrorAction SilentlyContinue
    if ($searchResult) {
        $vncFound = $true
        $vncPath = $searchResult.Source
        Write-Host "  [OK] TightVNC trovato nel PATH: $vncPath" -ForegroundColor Green
    } else {
        Write-Host "  [ERR] TightVNC Viewer non trovato nel sistema" -ForegroundColor Red
        Write-Host ""
        Write-Host "  [INFO] Per installare TightVNC Viewer:" -ForegroundColor Yellow
        Write-Host "    1. Scarica da: https://www.tightvnc.com/download.php" -ForegroundColor Yellow
        Write-Host "    2. Installa solo il Viewer (non il Server)" -ForegroundColor Yellow
        Write-Host "    3. Oppure usa: winget install TightVNC.TightVNC" -ForegroundColor Yellow
    }
}

# ========================================================================
# 2. VERIFICA PING ROBOT
# ========================================================================
Write-Host ""
Write-Host "[2/3] Verifica ping robot ($ROBOT_IP)..." -ForegroundColor Yellow

try {
    $pingResult = Test-Connection -ComputerName $ROBOT_IP -Count 4 -ErrorAction Stop
    
    if ($pingResult) {
        $avgTime = ($pingResult | Measure-Object -Property ResponseTime -Average).Average
        Write-Host "  [OK] Robot raggiungibile!" -ForegroundColor Green
        Write-Host "  [INFO] Tempo medio risposta: $([math]::Round($avgTime, 2)) ms" -ForegroundColor Cyan
        Write-Host "  [INFO] Pacchetti inviati: 4" -ForegroundColor Cyan
        Write-Host "  [INFO] Pacchetti ricevuti: $($pingResult.Count)" -ForegroundColor Cyan
    } else {
        Write-Host "  [ERR] Robot NON raggiungibile (nessuna risposta)" -ForegroundColor Red
    }
} catch {
    Write-Host "  [ERR] Errore ping: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "  [INFO] Verifica:" -ForegroundColor Yellow
    Write-Host "    - Il robot e acceso?" -ForegroundColor Yellow
    Write-Host "    - Il robot e sulla stessa rete?" -ForegroundColor Yellow
    Write-Host "    - Il firewall blocca il ping?" -ForegroundColor Yellow
}

# ========================================================================
# 3. VERIFICA PORTA VNC ROBOT
# ========================================================================
Write-Host ""
Write-Host "[3/3] Verifica porta VNC robot (${ROBOT_IP}:${VNC_PORT})..." -ForegroundColor Yellow

try {
    $tcpClient = New-Object System.Net.Sockets.TcpClient
    $connect = $tcpClient.BeginConnect($ROBOT_IP, $VNC_PORT, $null, $null)
    $wait = $connect.AsyncWaitHandle.WaitOne(3000, $false)
    
    if ($wait) {
        try {
            $tcpClient.EndConnect($connect)
            Write-Host "  [OK] Porta VNC aperta e raggiungibile!" -ForegroundColor Green
            Write-Host "  [INFO] Puoi connetterti con:" -ForegroundColor Cyan
            if ($vncFound) {
                Write-Host "    $vncPath $ROBOT_IP::$VNC_PORT" -ForegroundColor White
            } else {
                Write-Host "    vncviewer $ROBOT_IP::$VNC_PORT" -ForegroundColor White
                Write-Host "    oppure usa TightVNC Viewer e inserisci: $ROBOT_IP::$VNC_PORT" -ForegroundColor White
            }
        } catch {
            Write-Host "  [ERR] Connessione fallita: $($_.Exception.Message)" -ForegroundColor Red
        }
    } else {
        Write-Host "  [ERR] Porta VNC non raggiungibile (timeout)" -ForegroundColor Red
        Write-Host "  [INFO] Verifica:" -ForegroundColor Yellow
        Write-Host "    - VNC Server attivo sul robot?" -ForegroundColor Yellow
        Write-Host "    - Firewall non blocca la porta $VNC_PORT?" -ForegroundColor Yellow
    }
    
    $tcpClient.Close()
} catch {
    Write-Host "  [ERR] Errore verifica porta: $($_.Exception.Message)" -ForegroundColor Red
}

# ========================================================================
# RIEPILOGO
# ========================================================================
Write-Host ""
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "RIEPILOGO" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan

Write-Host "[STATUS] TightVNC Viewer: $(if ($vncFound) { '[OK] Installato' } else { '[ERR] Non trovato' })" -ForegroundColor $(if ($vncFound) { 'Green' } else { 'Red' })

$pingOk = $false
try {
    $pingTest = Test-Connection -ComputerName $ROBOT_IP -Count 1 -Quiet -ErrorAction Stop
    $pingOk = $pingTest
} catch {
    $pingOk = $false
}
Write-Host "[STATUS] Ping Robot: $(if ($pingOk) { '[OK] Raggiungibile' } else { '[ERR] Non raggiungibile' })" -ForegroundColor $(if ($pingOk) { 'Green' } else { 'Red' })

$vncPortOk = $false
try {
    $tcpClient = New-Object System.Net.Sockets.TcpClient
    $connect = $tcpClient.BeginConnect($ROBOT_IP, $VNC_PORT, $null, $null)
    $wait = $connect.AsyncWaitHandle.WaitOne(2000, $false)
    if ($wait) {
        $tcpClient.EndConnect($connect)
        $vncPortOk = $true
    }
    $tcpClient.Close()
} catch {
    $vncPortOk = $false
}
Write-Host "[STATUS] Porta VNC: $(if ($vncPortOk) { '[OK] Aperta' } else { '[ERR] Chiusa/Non raggiungibile' })" -ForegroundColor $(if ($vncPortOk) { 'Green' } else { 'Red' })

Write-Host ""
if ($vncFound -and $pingOk -and $vncPortOk) {
    Write-Host "[SUCCESS] Tutto OK! Puoi connetterti via VNC:" -ForegroundColor Green
    if ($vncPath) {
        Write-Host "  Start-Process '$vncPath' -ArgumentList '$ROBOT_IP::$VNC_PORT'" -ForegroundColor White
    } else {
        Write-Host "  vncviewer $ROBOT_IP::$VNC_PORT" -ForegroundColor White
    }
} else {
    Write-Host "[INFO] Risolvi i problemi sopra prima di connetterti via VNC" -ForegroundColor Yellow
}
