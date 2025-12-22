# Script PowerShell per organizzare repository
# Sposta file non essenziali in archive/

$archiveDir = "archive"
$essentialFiles = @(
    "remote_ur_control",
    "switch_controller.py",
    "attiva_controller.py",
    "attiva_controller.sh",
    "start_web_interface.py",
    "README.md",
    "ORGANIZZA_REPOSITORY.md",
    "SOLUZIONE_TIMEOUT_SWITCH_CONTROLLER.txt",
    "SOLUZIONE_FORWARD_VELOCITY_CONTROLLER_BLOCCATO.txt",
    "RIEPILOGO_FIX_FORWARD_VELOCITY.txt",
    "verifica_log_driver_ros2.ps1",
    "verifica_vnc_e_poweron.py",
    "verifica_driver_ros2_e_porta_50002.ps1",
    "verifica_connessione_robot_server.py",
    "verifica_tightvnc_e_robot.ps1",
    "ros2_bridge_fixed.py",
    "test",
    "archive",
    ".git",
    "organizza_repository.ps1"
)

Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "ORGANIZZAZIONE REPOSITORY" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""

# Crea cartella archive se non esiste
if (-not (Test-Path $archiveDir)) {
    New-Item -ItemType Directory -Path $archiveDir | Out-Null
    Write-Host "✅ Creata cartella $archiveDir" -ForegroundColor Green
}

# Ottieni tutti i file
$allFiles = Get-ChildItem -File

# Filtra file da spostare
$filesToMove = @()
foreach ($file in $allFiles) {
    $isEssential = $false
    foreach ($essential in $essentialFiles) {
        if ($file.Name -eq $essential -or $file.Name -like "*$essential*") {
            $isEssential = $true
            break
        }
    }
    if (-not $isEssential) {
        $filesToMove += $file
    }
}

Write-Host "📁 File da spostare in archive/: $($filesToMove.Count)" -ForegroundColor Yellow
Write-Host ""

# Sposta file
$moved = 0
foreach ($file in $filesToMove) {
    $dest = Join-Path $archiveDir $file.Name
    if (-not (Test-Path $dest)) {
        Move-Item -Path $file.FullName -Destination $dest -Force
        Write-Host "  📦 $($file.Name)" -ForegroundColor Gray
        $moved++
    } else {
        Write-Host "  ⚠️  $($file.Name) già esiste in archive/" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "✅ Spostati $moved file in archive/" -ForegroundColor Green
Write-Host ""
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host "REPOSITORY ORGANIZZATA!" -ForegroundColor Cyan
Write-Host ("=" * 70) -ForegroundColor Cyan
Write-Host ""
Write-Host "File essenziali mantenuti:" -ForegroundColor Yellow
foreach ($file in $essentialFiles) {
    if (Test-Path $file) {
        Write-Host "  ✅ $file" -ForegroundColor Green
    }
}


