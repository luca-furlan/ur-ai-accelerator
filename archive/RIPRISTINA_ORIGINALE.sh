#!/bin/bash
# Script per ripristinare web_interface.py originale se necessario

cd ~/MekoAiAccelerator

echo "=================================================================================="
echo "RIPRISTINO web_interface.py ORIGINALE"
echo "=================================================================================="
echo ""

# Trova backup più recente
BACKUP_FILE=$(ls -t remote_ur_control/web_interface.py.backup* 2>/dev/null | head -1)

if [ -z "$BACKUP_FILE" ]; then
    echo "❌ Nessun backup trovato"
    echo ""
    echo "I backup dovrebbero essere in:"
    echo "  remote_ur_control/web_interface.py.backup*"
    exit 1
fi

echo "Backup trovato: $BACKUP_FILE"
echo ""
read -p "Ripristinare web_interface.py originale? (y/n) " -n 1 -r
echo

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Operazione annullata"
    exit 0
fi

# Ripristina
cp "$BACKUP_FILE" remote_ur_control/web_interface.py
echo "✅ web_interface.py ripristinato da: $BACKUP_FILE"
echo ""
echo "Ora puoi usare il vecchio script:"
echo "  ./avvia_web_interface_joystick.sh"




