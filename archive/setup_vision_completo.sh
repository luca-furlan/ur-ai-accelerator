#!/bin/bash

# ================================================================================
# SETUP COMPLETO VISION SYSTEM - TUTTO IN UNO
# Deploy + Installa + Testa
# Da eseguire sull'AI Accelerator
# ================================================================================

set -e

echo "=================================================================================="
echo "🚀 SETUP COMPLETO VISION SYSTEM"
echo "=================================================================================="
echo ""

# ========================================
# 1. Verifica directory
# ========================================
if [ ! -f "vision_yolo_detector.py" ]; then
    echo "❌ File vision non trovati in questa directory"
    echo ""
    echo "Assicurati di:"
    echo "  1. Fare deploy da Windows (bash deploy_vision_completo.sh)"
    echo "  2. Essere in ~/MekoAiAccelerator"
    exit 1
fi

echo "✅ File vision trovati"
echo ""

# ========================================
# 2. Rendi eseguibili
# ========================================
echo "[1/3] Configurazione permessi..."
chmod +x *.sh *.py 2>/dev/null || true
echo "✅ Permessi configurati"
echo ""

# ========================================
# 3. Installa dipendenze
# ========================================
echo "[2/3] Installazione dipendenze..."
echo ""

if [ -f "installa_dipendenze_vision.sh" ]; then
    bash installa_dipendenze_vision.sh
else
    echo "❌ installa_dipendenze_vision.sh non trovato"
    exit 1
fi

echo ""

# ========================================
# 4. Test sistema
# ========================================
echo "[3/3] Test sistema..."
echo ""

if [ -f "test_vision_system.py" ]; then
    python3 test_vision_system.py
else
    echo "⚠️  test_vision_system.py non trovato (skip test)"
fi

echo ""
echo "=================================================================================="
echo "✅ SETUP COMPLETATO!"
echo "=================================================================================="
echo ""
echo "Sistema pronto all'uso!"
echo ""
echo "Per avviare:"
echo "  ./avvia_web_interface_con_vision.sh"
echo ""
echo "Accesso da browser:"
echo "  http://$(hostname -I | awk '{print $1}'):8080"
echo ""
echo "=================================================================================="




