# Script PowerShell per verificare driver ROS2 e porta 50002 sul server
# Esegui: .\verifica_driver_ros2_e_porta_50002.ps1

$SERVER_IP = "192.168.10.191"
$EXTERNAL_CONTROL_PORT = 50002
$SSH_USER = "lab"

Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "VERIFICA DRIVER ROS2 E PORTA 50002" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host ""

# ========================================================================
# 1. VERIFICA CONNESSIONE SERVER
# ========================================================================
Write-Host "[1/4] Verifica connessione server ($SERVER_IP)..." -ForegroundColor Yellow

try {
    $pingResult = Test-Connection -ComputerName $SERVER_IP -Count 2 -Quiet -ErrorAction Stop
    if ($pingResult) {
        Write-Host "  [OK] Server raggiungibile!" -ForegroundColor Green
    } else {
        Write-Host "  [ERR] Server NON raggiungibile" -ForegroundColor Red
        Write-Host "  [INFO] Verifica che il server sia acceso e sulla stessa rete" -ForegroundColor Yellow
        exit 1
    }
} catch {
    Write-Host "  [ERR] Errore ping: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}

# ========================================================================
# 2. VERIFICA PORTA 50002 DAL TUO PC
# ========================================================================
Write-Host ""
Write-Host "[2/4] Verifica porta 50002 dal tuo PC..." -ForegroundColor Yellow

try {
    $tcpClient = New-Object System.Net.Sockets.TcpClient
    $connect = $tcpClient.BeginConnect($SERVER_IP, $EXTERNAL_CONTROL_PORT, $null, $null)
    $wait = $connect.AsyncWaitHandle.WaitOne(3000, $false)
    
    if ($wait) {
        try {
            $tcpClient.EndConnect($connect)
            Write-Host "  [OK] Porta 50002 aperta e raggiungibile!" -ForegroundColor Green
            Write-Host "  [INFO] Il driver ROS2 e attivo e in ascolto" -ForegroundColor Cyan
            $portOpen = $true
        } catch {
            Write-Host "  [ERR] Connessione fallita: $($_.Exception.Message)" -ForegroundColor Red
            $portOpen = $false
        }
    } else {
        Write-Host "  [ERR] Porta 50002 NON raggiungibile (timeout)" -ForegroundColor Red
        Write-Host "  [INFO] Il driver ROS2 probabilmente NON e attivo" -ForegroundColor Yellow
        $portOpen = $false
    }
    
    $tcpClient.Close()
} catch {
    Write-Host "  [ERR] Errore verifica porta: $($_.Exception.Message)" -ForegroundColor Red
    $portOpen = $false
}

# ========================================================================
# 3. VERIFICA PROCESSI ROS2 SUL SERVER (via SSH)
# ========================================================================
Write-Host ""
Write-Host "[3/4] Verifica processi ROS2 sul server..." -ForegroundColor Yellow

$sshCommands = @"
cd ~/MekoAiAccelerator
echo '=== PROCESSI ROS2 ==='
ps aux | grep -E 'ur_ros2_control|ur_control.launch|ros2 launch' | grep -v grep || echo 'Nessun processo ROS2 trovato'
echo ''
echo '=== PORTA 50002 ==='
netstat -tuln | grep 50002 || echo 'Porta 50002 non in ascolto'
echo ''
echo '=== STATO DRIVER ==='
if pgrep -f 'ur_ros2_control' > /dev/null; then
    echo '[OK] Driver ROS2 attivo'
else
    echo '[ERR] Driver ROS2 NON attivo'
fi
"@

try {
    Write-Host "  [INFO] Connessione SSH al server..." -ForegroundColor Cyan
    $sshResult = ssh "${SSH_USER}@${SERVER_IP}" $sshCommands 2>&1
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host $sshResult
    } else {
        Write-Host "  [WARN] Impossibile connettersi via SSH o comando fallito" -ForegroundColor Yellow
        Write-Host "  [INFO] Connettiti manualmente: ssh ${SSH_USER}@${SERVER_IP}" -ForegroundColor Yellow
    }
} catch {
    Write-Host "  [WARN] Errore SSH: $($_.Exception.Message)" -ForegroundColor Yellow
    Write-Host "  [INFO] Connettiti manualmente: ssh ${SSH_USER}@${SERVER_IP}" -ForegroundColor Yellow
}

# ========================================================================
# 4. SPIEGAZIONE E SOLUZIONE
# ========================================================================
Write-Host ""
Write-Host "[4/4] Spiegazione problema..." -ForegroundColor Yellow

if (-not $portOpen) {
    Write-Host "  [PROBLEMA] Il robot non riesce a connettersi alla porta 50002" -ForegroundColor Red
    Write-Host ""
    Write-Host "  [SPIEGAZIONE]:" -ForegroundColor Cyan
    Write-Host "    Il robot UR cerca di connettersi a:" -ForegroundColor White
    Write-Host "      IP: $SERVER_IP" -ForegroundColor White
    Write-Host "      Porta: $EXTERNAL_CONTROL_PORT" -ForegroundColor White
    Write-Host ""
    Write-Host "    Questa porta viene aperta dal driver ROS2 quando viene avviato." -ForegroundColor White
    Write-Host "    Se il driver ROS2 NON e attivo, la porta non e in ascolto e" -ForegroundColor White
    Write-Host "    il robot non riesce a connettersi." -ForegroundColor White
    Write-Host ""
    Write-Host "  [SOLUZIONE]:" -ForegroundColor Green
    Write-Host "    1. Connettiti al server: ssh ${SSH_USER}@${SERVER_IP}" -ForegroundColor White
    Write-Host "    2. Avvia il driver ROS2:" -ForegroundColor White
    Write-Host "       cd ~/MekoAiAccelerator" -ForegroundColor White
    Write-Host "       export UR_ROBOT_IP=192.168.10.194" -ForegroundColor White
    Write-Host "       ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false initial_joint_controller:=forward_velocity_controller" -ForegroundColor White
    Write-Host ""
    Write-Host "    3. Oppure usa la web interface:" -ForegroundColor White
    Write-Host "       http://${SERVER_IP}:8080" -ForegroundColor White
    Write-Host "       Clicca 'Avvia Driver' nello step A del wizard" -ForegroundColor White
    Write-Host ""
    Write-Host "    4. Verifica che la porta sia aperta:" -ForegroundColor White
    Write-Host "       netstat -tuln | grep 50002" -ForegroundColor White
} else {
    Write-Host "  [OK] Porta 50002 aperta - driver ROS2 attivo!" -ForegroundColor Green
    Write-Host "  [INFO] Il robot dovrebbe riuscire a connettersi" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "  [VERIFICA]:" -ForegroundColor Yellow
    Write-Host "    Se il robot ancora non si connette, verifica:" -ForegroundColor White
    Write-Host "    1. Che il programma sul robot abbia l'IP corretto: $SERVER_IP" -ForegroundColor White
    Write-Host "    2. Che la porta sia corretta: $EXTERNAL_CONTROL_PORT" -ForegroundColor White
    Write-Host "    3. Che Remote Control sia attivo sul Teach Pendant" -ForegroundColor White
}

# ========================================================================
# RIEPILOGO
# ========================================================================
Write-Host ""
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "RIEPILOGO" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan

Write-Host "[STATUS] Server: [OK] Raggiungibile" -ForegroundColor Green
Write-Host "[STATUS] Porta 50002: $(if ($portOpen) { '[OK] Aperta' } else { '[ERR] Chiusa - Driver ROS2 non attivo' })" -ForegroundColor $(if ($portOpen) { 'Green' } else { 'Red' })

Write-Host ""
if (-not $portOpen) {
    Write-Host "[AZIONE RICHIESTA] Avvia il driver ROS2 sul server!" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Comando rapido:" -ForegroundColor Cyan
    Write-Host "  ssh ${SSH_USER}@${SERVER_IP} 'cd ~/MekoAiAccelerator && export UR_ROBOT_IP=192.168.10.194 && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false initial_joint_controller:=forward_velocity_controller &'" -ForegroundColor White
}
