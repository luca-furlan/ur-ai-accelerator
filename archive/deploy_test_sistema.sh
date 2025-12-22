#!/bin/bash
# Script per trasferire e avviare test sistema sulla macchina AI Accelerator

AI_ACCELERATOR_IP="192.168.10.191"
AI_ACCELERATOR_USER="lab"
AI_ACCELERATOR_PASS="easybot"
REMOTE_DIR="~/MekoAiAccelerator"

echo "=========================================="
echo "DEPLOY TEST SISTEMA SU AI ACCELERATOR"
echo "=========================================="
echo ""

# Verifica connessione
echo "1. Verifica connessione a AI Accelerator..."
if ping -c 1 -W 2 $AI_ACCELERATOR_IP > /dev/null 2>&1; then
    echo "   ✅ AI Accelerator raggiungibile"
else
    echo "   ❌ AI Accelerator NON raggiungibile"
    exit 1
fi

# File da trasferire
FILES_TO_TRANSFER=(
    "test_sistema_completo.py"
    "quick_check_sistema.py"
    "test/run_all_tests.py"
    "test/test_connettivita_robot.py"
    "test/test_ros2_driver.py"
    "test/test_camera_orbbec.py"
    "test/test_mujoco.py"
    "test/test_ai_components.py"
    "test/test_web_interface.py"
    "test/__init__.py"
    "test/README.md"
    "GUIDA_TEST_COMPLETA.md"
    "RIEPILOGO_TEST_SISTEMA.md"
    "QUICK_REFERENCE_TEST.md"
)

echo ""
echo "2. Trasferimento file..."

# Usa scp se disponibile, altrimenti suggerisci rsync o manuale
if command -v scp > /dev/null 2>&1; then
    echo "   Uso SCP per trasferire file..."
    
    # Crea directory test su remoto se non esiste
    sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP \
        "mkdir -p $REMOTE_DIR/test" 2>/dev/null
    
    # Trasferisci file
    for file in "${FILES_TO_TRANSFER[@]}"; do
        if [ -f "$file" ]; then
            echo "   📤 Trasferisco: $file"
            sshpass -p "$AI_ACCELERATOR_PASS" scp -o StrictHostKeyChecking=no \
                "$file" $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/"$file" 2>/dev/null
        else
            echo "   ⚠️  File non trovato: $file"
        fi
    done
    
    echo "   ✅ File trasferiti"
else
    echo "   ⚠️  SCP non disponibile"
    echo "   Trasferisci manualmente i file o installa sshpass/scp"
    echo ""
    echo "   Comandi manuali:"
    echo "   scp test_sistema_completo.py $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/"
    echo "   scp -r test/ $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/"
fi

echo ""
echo "3. Rendi eseguibili i file Python..."
sshpass -p "$AI_ACCELERATOR_PASS" ssh -o StrictHostKeyChecking=no $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP << 'EOF'
    cd ~/MekoAiAccelerator
    chmod +x test_sistema_completo.py quick_check_sistema.py
    chmod +x test/*.py
    echo "   ✅ File resi eseguibili"
EOF

echo ""
echo "=========================================="
echo "DEPLOY COMPLETATO!"
echo "=========================================="
echo ""
echo "Per avviare i test sulla macchina AI Accelerator:"
echo ""
echo "1. Connettiti via SSH:"
echo "   ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo ""
echo "2. Vai nella directory:"
echo "   cd ~/MekoAiAccelerator"
echo ""
echo "3. Avvia test:"
echo "   python3 quick_check_sistema.py          # Quick check"
echo "   python3 test_sistema_completo.py        # Verifica completa"
echo "   python3 test/run_all_tests.py           # Tutti i test"
echo ""
echo "=========================================="











