#!/bin/bash

# Script per installare e configurare UFW manualmente
# Esegui questo script se FIX_COMPLETO_AUTOMATICO.sh non può installare UFW automaticamente

echo "=================================================================================="
echo "🔧 INSTALLAZIONE MANUALE UFW"
echo "=================================================================================="
echo ""

PORTS=(50001 50002 50003 50004)

echo "1. Aggiornamento repository..."
sudo apt-get update

echo ""
echo "2. Installazione UFW..."
sudo apt-get install -y ufw

echo ""
echo "3. Apertura porte necessarie..."
for PORT in "${PORTS[@]}"; do
    echo "   Aprendo porta $PORT..."
    sudo ufw allow $PORT
done

echo ""
echo "4. Abilitazione UFW..."
echo "y" | sudo ufw enable

echo ""
echo "5. Verifica stato UFW..."
sudo ufw status verbose

echo ""
echo "=================================================================================="
echo "✅ UFW installato e configurato!"
echo "=================================================================================="
echo ""
echo "Porte aperte: 50001, 50002, 50003, 50004"
echo ""

