# Script PowerShell per connettersi, fixare e avviare web interface
# Esegui da PowerShell: .\fix_e_avvia.ps1

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$ROBOT_IP = "192.168.10.194"

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "CONNESSIONE E AVVIO WEB INTERFACE" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Comando completo da eseguire
$script = @"
#!/bin/bash
set -e

echo '=========================================='
echo 'FIX E AVVIO WEB INTERFACE'
echo '=========================================='
echo ''

# 1. Ferma processi esistenti
echo '1. Fermo processi esistenti...'
pkill -f web_interface 2>/dev/null || true
sleep 2

# 2. Vai nella directory
cd ~/MekoAiAccelerator || { echo 'ERRORE: Directory non trovata!'; exit 1; }

# 3. Verifica file
if [ ! -f remote_ur_control/web_interface.py ]; then
    echo 'ERRORE: web_interface.py non trovato!'
    exit 1
fi
echo '✅ File presente'

# 4. Verifica Flask
if ! python3 -c 'import flask' 2>/dev/null; then
    echo '⚠️  Flask non trovato - installazione...'
    pip3 install --user flask flask-cors
fi
echo '✅ Flask OK'

# 5. Verifica robot
export UR_ROBOT_IP=$ROBOT_IP
if ping -c 1 $ROBOT_IP > /dev/null 2>&1; then
    echo '✅ Robot raggiungibile'
else
    echo '⚠️  Robot NON raggiungibile (continua comunque)'
fi

# 6. Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || echo '⚠️  ROS2 non trovato (userà socket fallback)'
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# 7. Libera porta 8080
echo ''
echo '7. Verifica porta 8080...'
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

# 8. Configura
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# 9. Avvia web interface
echo ''
echo '8. Avvio web interface...'
cd ~/MekoAiAccelerator
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
WEB_PID=`$!
sleep 3

# 10. Verifica
if ps -p `$WEB_PID > /dev/null 2>&1; then
    echo ''
    echo '=========================================='
    echo '✅ WEB INTERFACE AVVIATA!'
    echo '=========================================='
    echo ''
    echo '🌐 Connettiti qui:'
    echo '   http://192.168.10.191:8080'
    echo ''
    echo '📋 Log: tail -f /tmp/web_interface.log'
    echo '📋 PID: '`$WEB_PID
    echo ''
    echo 'Per vedere i log in tempo reale:'
    echo '  tail -f /tmp/web_interface.log'
    echo ''
else
    echo '❌ Errore avvio!'
    echo 'Log:'
    cat /tmp/web_interface.log
    exit 1
fi
"@

Write-Host "Invio script all'AI Accelerator..." -ForegroundColor Yellow

# Esegui script via SSH
$command = "bash -s"
$scriptBytes = [System.Text.Encoding]::UTF8.GetBytes($script)
$scriptBase64 = [Convert]::ToBase64String($scriptBytes)

# Usa plink se disponibile, altrimenti ssh
try {
    $result = & ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15 "$AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" $script 2>&1
    Write-Host $result
} catch {
    Write-Host "Errore connessione SSH. Esegui manualmente:" -ForegroundColor Red
    Write-Host ""
    Write-Host "ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Yellow
    Write-Host "Poi copia e incolla questo script:" -ForegroundColor Yellow
    Write-Host ""
    Write-Host $script -ForegroundColor Green
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "Se tutto ok, connettiti a:" -ForegroundColor Green
Write-Host "http://192.168.10.191:8080" -ForegroundColor Yellow
Write-Host "==========================================" -ForegroundColor Cyan

