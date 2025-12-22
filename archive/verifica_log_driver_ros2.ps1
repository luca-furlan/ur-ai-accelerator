# Script PowerShell per verificare log driver ROS2 sul server
# Esegui: .\verifica_log_driver_ros2.ps1

$SERVER_IP = "192.168.10.191"
$SSH_USER = "lab"

Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "VERIFICA LOG DRIVER ROS2 E FORWARD_VELOCITY_CONTROLLER" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host ""

Write-Host "[1/5] Verifica log driver ROS2..." -ForegroundColor Yellow
$logCmd = "tail -100 /tmp/ros2_driver.log 2>/dev/null || echo 'File log non trovato'"
ssh "${SSH_USER}@${SERVER_IP}" $logCmd

Write-Host ""
Write-Host "[2/5] Verifica processi ROS2..." -ForegroundColor Yellow
$procCmd = "ps aux | grep -E 'ur_ros2_control|ur_control.launch|ros2 launch' | grep -v grep || echo 'Nessun processo ROS2 trovato'"
ssh "${SSH_USER}@${SERVER_IP}" $procCmd

Write-Host ""
Write-Host "[3/5] Verifica stato controller..." -ForegroundColor Yellow
$controllerCmd = "source /opt/ros/humble/setup.bash 2>/dev/null && source ~/ros2_ws/install/setup.bash 2>/dev/null && ros2 control list_controllers 2>&1 | head -30 || echo 'Errore comando'"
ssh "${SSH_USER}@${SERVER_IP}" $controllerCmd

Write-Host ""
Write-Host "[4/5] Verifica servizi controller_manager..." -ForegroundColor Yellow
$serviceCmd = "source /opt/ros/humble/setup.bash 2>/dev/null && source ~/ros2_ws/install/setup.bash 2>/dev/null && ros2 service list | grep controller_manager || echo 'Nessun servizio trovato'"
ssh "${SSH_USER}@${SERVER_IP}" $serviceCmd

Write-Host ""
Write-Host "[5/5] Verifica errori recenti..." -ForegroundColor Yellow
$errorCmd = "journalctl --no-pager -n 30 2>/dev/null | grep -i -E 'error|fail|timeout|controller' | tail -20 || dmesg | tail -20 | grep -i error || echo 'Nessun errore trovato'"
ssh "${SSH_USER}@${SERVER_IP}" $errorCmd

Write-Host ""
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "Per vedere tutti i log in tempo reale:" -ForegroundColor Yellow
Write-Host "  ssh ${SSH_USER}@${SERVER_IP}" -ForegroundColor White
Write-Host "  tail -f /tmp/ros2_driver.log" -ForegroundColor White
