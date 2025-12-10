#!/bin/bash
# Script per avviare test sistema sulla macchina AI Accelerator via SSH

AI_ACCELERATOR_IP="192.168.10.191"
AI_ACCELERATOR_USER="lab"
AI_ACCELERATOR_PASS="easybot"
REMOTE_DIR="~/MekoAiAccelerator"

# Tipo di test da eseguire (default: quick_check)
TEST_TYPE="${1:-quick_check}"

echo "=========================================="
echo "AVVIO TEST SISTEMA SU AI ACCELERATOR"
echo "=========================================="
echo ""

# Verifica connessione
echo "Verifica connessione..."
if ping -c 1 -W 2 $AI_ACCELERATOR_IP > /dev/null 2>&1; then
    echo "✅ AI Accelerator raggiungibile"
else
    echo "❌ AI Accelerator NON raggiungibile"
    exit 1
fi

echo ""
echo "Esecuzione test: $TEST_TYPE"
echo ""

case $TEST_TYPE in
    quick)
    quick_check)
        echo "Eseguo quick check..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 quick_check_sistema.py"
        ;;
    
    completo|full|all)
        echo "Eseguo verifica completa..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test_sistema_completo.py"
        ;;
    
    tutti|all_tests)
        echo "Eseguo tutti i test funzionali..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/run_all_tests.py"
        ;;
    
    connettivita|robot)
        echo "Test connettività robot..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/test_connettivita_robot.py"
        ;;
    
    ros2|driver)
        echo "Test ROS2 driver..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/test_ros2_driver.py"
        ;;
    
    camera|orbbec)
        echo "Test camera Orbbec..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/test_camera_orbbec.py"
        ;;
    
    mujoco)
        echo "Test MuJoCo..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/test_mujoco.py"
        ;;
    
    ai)
        echo "Test componenti AI..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/test_ai_components.py"
        ;;
    
    web|interface)
        echo "Test web interface..."
        sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
            "cd $REMOTE_DIR && python3 test/test_web_interface.py"
        ;;
    
    *)
        echo "Uso: $0 [quick|completo|tutti|connettivita|ros2|camera|mujoco|ai|web]"
        echo ""
        echo "Esempi:"
        echo "  $0 quick          # Quick check"
        echo "  $0 completo      # Verifica completa"
        echo "  $0 tutti         # Tutti i test funzionali"
        echo "  $0 connettivita  # Test connettività robot"
        echo "  $0 ros2          # Test ROS2 driver"
        echo "  $0 camera        # Test camera Orbbec"
        echo "  $0 mujoco        # Test MuJoCo"
        echo "  $0 ai            # Test componenti AI"
        echo "  $0 web           # Test web interface"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Test completato!"
echo "=========================================="





