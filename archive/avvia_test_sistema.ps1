# Script PowerShell per avviare test sistema sulla macchina AI Accelerator

$AI_ACCELERATOR_IP = "192.168.10.191"
$AI_ACCELERATOR_USER = "lab"
$AI_ACCELERATOR_PASS = "easybot"
$REMOTE_DIR = "~/MekoAiAccelerator"

# Tipo di test (default: quick_check)
$TEST_TYPE = if ($args.Count -gt 0) { $args[0] } else { "quick_check" }

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "AVVIO TEST SISTEMA SU AI ACCELERATOR" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Verifica connessione
Write-Host "Verifica connessione..." -ForegroundColor Yellow
$ping = Test-Connection -ComputerName $AI_ACCELERATOR_IP -Count 1 -Quiet
if ($ping) {
    Write-Host "✅ AI Accelerator raggiungibile" -ForegroundColor Green
} else {
    Write-Host "❌ AI Accelerator NON raggiungibile" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Esecuzione test: $TEST_TYPE" -ForegroundColor Yellow
Write-Host ""

# Comando SSH da eseguire
$sshCommand = switch ($TEST_TYPE) {
    { $_ -in "quick", "quick_check" } {
        "cd $REMOTE_DIR && python3 quick_check_sistema.py"
    }
    { $_ -in "completo", "full", "all" } {
        "cd $REMOTE_DIR && python3 test_sistema_completo.py"
    }
    { $_ -in "tutti", "all_tests" } {
        "cd $REMOTE_DIR && python3 test/run_all_tests.py"
    }
    { $_ -in "connettivita", "robot" } {
        "cd $REMOTE_DIR && python3 test/test_connettivita_robot.py"
    }
    { $_ -in "ros2", "driver" } {
        "cd $REMOTE_DIR && python3 test/test_ros2_driver.py"
    }
    { $_ -in "camera", "orbbec" } {
        "cd $REMOTE_DIR && python3 test/test_camera_orbbec.py"
    }
    "mujoco" {
        "cd $REMOTE_DIR && python3 test/test_mujoco.py"
    }
    "ai" {
        "cd $REMOTE_DIR && python3 test/test_ai_components.py"
    }
    { $_ -in "web", "interface" } {
        "cd $REMOTE_DIR && python3 test/test_web_interface.py"
    }
    default {
        Write-Host "Uso: .\avvia_test_sistema.ps1 [quick|completo|tutti|connettivita|ros2|camera|mujoco|ai|web]" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Esempi:" -ForegroundColor Cyan
        Write-Host "  .\avvia_test_sistema.ps1 quick          # Quick check"
        Write-Host "  .\avvia_test_sistema.ps1 completo      # Verifica completa"
        Write-Host "  .\avvia_test_sistema.ps1 tutti         # Tutti i test funzionali"
        Write-Host "  .\avvia_test_sistema.ps1 connettivita  # Test connettività robot"
        Write-Host "  .\avvia_test_sistema.ps1 ros2          # Test ROS2 driver"
        Write-Host "  .\avvia_test_sistema.ps1 camera        # Test camera Orbbec"
        Write-Host "  .\avvia_test_sistema.ps1 mujoco        # Test MuJoCo"
        Write-Host "  .\avvia_test_sistema.ps1 ai            # Test componenti AI"
        Write-Host "  .\avvia_test_sistema.ps1 web           # Test web interface"
        exit 1
    }
}

# Esegui comando SSH
Write-Host "Esecuzione comando SSH..." -ForegroundColor Yellow
Write-Host ""

# Usa plink (PuTTY) se disponibile, altrimenti suggerisci installazione
if (Get-Command plink -ErrorAction SilentlyContinue) {
    $plinkCommand = "echo $AI_ACCELERATOR_PASS | plink -ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP -pw $AI_ACCELERATOR_PASS `"$sshCommand`""
    Invoke-Expression $plinkCommand
} elseif (Get-Command ssh -ErrorAction SilentlyContinue) {
    # Prova con ssh normale (richiede configurazione SSH key o password interattiva)
    Write-Host "⚠️  Usando SSH standard (potrebbe richiedere password)" -ForegroundColor Yellow
    Write-Host "   Comando da eseguire manualmente:" -ForegroundColor Yellow
    Write-Host "   ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor Cyan
    Write-Host "   $sshCommand" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "   Oppure installa PuTTY/plink per esecuzione automatica" -ForegroundColor Yellow
} else {
    Write-Host "❌ SSH non disponibile" -ForegroundColor Red
    Write-Host ""
    Write-Host "Opzioni:" -ForegroundColor Yellow
    Write-Host "1. Installa OpenSSH: Add-WindowsCapability -Online -Name OpenSSH.Client" -ForegroundColor Cyan
    Write-Host "2. Installa PuTTY (include plink.exe)" -ForegroundColor Cyan
    Write-Host "3. Esegui manualmente:" -ForegroundColor Cyan
    Write-Host "   ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP" -ForegroundColor White
    Write-Host "   $sshCommand" -ForegroundColor White
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "Test completato!" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan











