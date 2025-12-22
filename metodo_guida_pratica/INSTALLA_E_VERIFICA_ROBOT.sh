#!/bin/bash

# Script completo: installa sshpass e verifica robot

echo "=================================================================================="
echo "🔧 INSTALLAZIONE SSHPASS E VERIFICA ROBOT"
echo "=================================================================================="
echo ""

# Installa sshpass se non presente
if ! command -v sshpass > /dev/null 2>&1; then
    echo "1. Installazione sshpass..."
    if command -v apt-get > /dev/null 2>&1; then
        sudo apt-get update
        sudo apt-get install -y sshpass
        if [ $? -eq 0 ]; then
            echo "   ✅ sshpass installato"
        else
            echo "   ❌ Impossibile installare sshpass (serve sudo)"
            echo "   Installa manualmente: sudo apt-get install sshpass"
            exit 1
        fi
    else
        echo "   ❌ Sistema non basato su apt-get"
        echo "   Installa sshpass manualmente"
        exit 1
    fi
else
    echo "1. sshpass già installato ✅"
fi
echo ""

# Esegui verifica robot
echo "2. Esecuzione verifica robot..."
echo ""
cd ~/MekoAiAccelerator/metodo_guida_pratica
if [ -f VERIFICA_ROBOT_SSH.sh ]; then
    bash VERIFICA_ROBOT_SSH.sh
else
    echo "   ❌ Script VERIFICA_ROBOT_SSH.sh non trovato"
fi








