# Script di Pulizia Sistema per Liberare Spazio su Disco C:
# ATTENZIONE: Eseguire con privilegi di amministratore per risultati migliori

Write-Host "=== PULIZIA SISTEMA - LIBERAZIONE SPAZIO DISCO ===" -ForegroundColor Cyan
Write-Host ""

# 1. Pulizia Cache Windows
Write-Host "1. Pulizia Cache Windows..." -ForegroundColor Yellow
try {
    # Pulizia file temporanei Windows
    Remove-Item -Path "$env:TEMP\*" -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "$env:LOCALAPPDATA\Temp\*" -Recurse -Force -ErrorAction SilentlyContinue
    
    # Pulizia Prefetch (richiede admin)
    $isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    if ($isAdmin) {
        Remove-Item -Path "$env:SystemRoot\Prefetch\*" -Force -ErrorAction SilentlyContinue
        Write-Host "   Prefetch pulito" -ForegroundColor Green
    }
    
    Write-Host "   Cache Windows pulita" -ForegroundColor Green
} catch {
    Write-Host "   Errore nella pulizia cache Windows: $_" -ForegroundColor Red
}

# 2. Pulizia Cache Cursor (solo cache, non impostazioni)
Write-Host "2. Pulizia Cache Cursor..." -ForegroundColor Yellow
try {
    $cursorCachePaths = @(
        "$env:LOCALAPPDATA\Cursor\Cache",
        "$env:LOCALAPPDATA\Cursor\CachedData",
        "$env:LOCALAPPDATA\Cursor\GPUCache",
        "$env:LOCALAPPDATA\Cursor\ShaderCache"
    )
    
    foreach ($path in $cursorCachePaths) {
        if (Test-Path $path) {
            $size = (Get-ChildItem $path -Recurse -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
            Remove-Item -Path "$path\*" -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "   Rimosso: $path ($([math]::Round($size/1MB,2)) MB)" -ForegroundColor Green
        }
    }
} catch {
    Write-Host "   Errore nella pulizia cache Cursor: $_" -ForegroundColor Red
}

# 3. Pulizia Cache Browser (se presente)
Write-Host "3. Pulizia Cache Browser..." -ForegroundColor Yellow
try {
    $chromeCache = "$env:LOCALAPPDATA\Google\Chrome\User Data\Default\Cache"
    if (Test-Path $chromeCache) {
        $size = (Get-ChildItem $chromeCache -Recurse -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
        Remove-Item -Path "$chromeCache\*" -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "   Cache Chrome pulita ($([math]::Round($size/1MB,2)) MB)" -ForegroundColor Green
    }
} catch {
    Write-Host "   Nessuna cache browser trovata o errore: $_" -ForegroundColor Yellow
}

# 4. Pulizia Windows Update (richiede admin)
Write-Host "4. Pulizia Windows Update..." -ForegroundColor Yellow
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if ($isAdmin) {
    try {
        # Usa Disk Cleanup Tool
        Start-Process -FilePath "cleanmgr.exe" -ArgumentList "/d C:" -NoNewWindow -Wait -ErrorAction SilentlyContinue
        Write-Host "   Disk Cleanup avviato" -ForegroundColor Green
    } catch {
        Write-Host "   Errore Disk Cleanup: $_" -ForegroundColor Red
    }
} else {
    Write-Host "   Richiesti privilegi amministratore per Disk Cleanup" -ForegroundColor Yellow
}

# 5. Pulizia Log Windows vecchi
Write-Host "5. Pulizia Log Windows..." -ForegroundColor Yellow
try {
    $logPaths = @(
        "$env:SystemRoot\Logs",
        "$env:SystemRoot\Temp"
    )
    
    foreach ($logPath in $logPaths) {
        if (Test-Path $logPath) {
            $oldLogs = Get-ChildItem $logPath -Recurse -File -ErrorAction SilentlyContinue | Where-Object {$_.LastWriteTime -lt (Get-Date).AddDays(-30)}
            if ($oldLogs) {
                $size = ($oldLogs | Measure-Object -Property Length -Sum).Sum
                $oldLogs | Remove-Item -Force -ErrorAction SilentlyContinue
                Write-Host "   Rimossi log vecchi da $logPath ($([math]::Round($size/1MB,2)) MB)" -ForegroundColor Green
            }
        }
    }
} catch {
    Write-Host "   Errore nella pulizia log: $_" -ForegroundColor Red
}

# 6. Verifica spazio finale
Write-Host ""
Write-Host "=== RISULTATO FINALE ===" -ForegroundColor Cyan
$disk = Get-CimInstance Win32_LogicalDisk | Where-Object {$_.DeviceID -eq "C:"}
Write-Host "Spazio Libero Disco C:: $([math]::Round($disk.FreeSpace/1GB,2)) GB ($([math]::Round(($disk.FreeSpace/$disk.Size)*100,2))%)" -ForegroundColor $(if ($disk.FreeSpace/$disk.Size -lt 0.1) {"Red"} else {"Green"})

Write-Host ""
Write-Host "Pulizia completata!" -ForegroundColor Green
Write-Host ""
Write-Host "SUGGERIMENTI:" -ForegroundColor Yellow
Write-Host "- Se lo spazio è ancora insufficiente, considera di:" -ForegroundColor Yellow
Write-Host "  1. Spostare file grandi su un altro disco" -ForegroundColor Yellow
Write-Host "  2. Disinstallare programmi non utilizzati" -ForegroundColor Yellow
Write-Host "  3. Usare Storage Sense di Windows (Impostazioni > Sistema > Archiviazione)" -ForegroundColor Yellow
Write-Host "  4. Eseguire questo script come Amministratore per pulizie più profonde" -ForegroundColor Yellow

