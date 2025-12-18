# ================================================================================
# VERIFICA STATO VNC SUL ROBOT
# ================================================================================

$ROBOT_IP = "192.168.10.191"
$SSH_USER = "lab"
$SSH_PASSWORD = "easybot"

Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host "VERIFICA STATO VNC SUL ROBOT" -ForegroundColor Cyan
Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host ""

# Verifica se plink (PuTTY) e' disponibile
$plinkPath = "C:\Program Files\PuTTY\plink.exe"
if (-not (Test-Path $plinkPath)) {
    Write-Host "[ATTENZIONE] plink.exe non trovato" -ForegroundColor Yellow
    Write-Host "Installa PuTTY da: https://www.putty.org/" -ForegroundColor White
    Write-Host ""
    Write-Host "Oppure usa SSH manualmente:" -ForegroundColor Yellow
    Write-Host "  ssh $SSH_USER@$ROBOT_IP" -ForegroundColor Cyan
    Write-Host "  vncserver -list" -ForegroundColor Cyan
    Write-Host "  ss -tlnp | grep 590" -ForegroundColor Cyan
    exit 1
}

Write-Host "[*] Connessione al robot..." -ForegroundColor Yellow

# Comando per verificare sessioni VNC
$vncListCmd = "vncserver -list"
$portCheckCmd = "ss -tlnp | grep 590 || netstat -tlnp | grep 590"

try {
    # Usa plink per eseguire comandi SSH
    $vncList = & $plinkPath -ssh -batch -pw $SSH_PASSWORD "$SSH_USER@$ROBOT_IP" $vncListCmd 2>&1
    $portCheck = & $plinkPath -ssh -batch -pw $SSH_PASSWORD "$SSH_USER@$ROBOT_IP" $portCheckCmd 2>&1
    
    Write-Host ""
    Write-Host "[*] Sessioni VNC attive:" -ForegroundColor Yellow
    Write-Host $vncList -ForegroundColor White
    Write-Host ""
    
    Write-Host "[*] Porte VNC in ascolto:" -ForegroundColor Yellow
    Write-Host $portCheck -ForegroundColor White
    Write-Host ""
    
    # Analizza output per trovare porte attive
    if ($vncList -match ":(\d+)") {
        $display = $matches[1]
        $port = "590$display"
        Write-Host "[OK] VNC attivo su display :$display (porta $port)" -ForegroundColor Green
        Write-Host ""
        Write-Host "[*] Connettiti con:" -ForegroundColor Cyan
        Write-Host "   $ROBOT_IP`:$port" -ForegroundColor White
        Write-Host "   Password: $SSH_PASSWORD" -ForegroundColor White
    } else {
        Write-Host "[ATTENZIONE] Nessuna sessione VNC trovata" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "[*] Per avviare VNC:" -ForegroundColor Yellow
        Write-Host "   ssh $SSH_USER@$ROBOT_IP" -ForegroundColor Cyan
        Write-Host "   vncserver :1 -geometry 1280x720 -depth 24 -localhost no" -ForegroundColor Cyan
    }
    
} catch {
    Write-Host "[ERRORE] Impossibile connettersi: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "[*] Verifica manualmente:" -ForegroundColor Yellow
    Write-Host "   ssh $SSH_USER@$ROBOT_IP" -ForegroundColor Cyan
    Write-Host "   vncserver -list" -ForegroundColor Cyan
}

Write-Host ""
