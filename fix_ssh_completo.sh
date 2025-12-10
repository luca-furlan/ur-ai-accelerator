#!/bin/bash
# Script completo per fixare SSH sulla macchina AI Accelerator

echo "================================================================================"
echo "FIX COMPLETO SERVIZIO SSH - AI ACCELERATOR"
echo "================================================================================"
echo

# 1. Installa SSH se mancante
echo "1. Verifica installazione SSH..."
if ! command -v sshd &> /dev/null; then
    echo "   ⚠️ SSH non installato, installo..."
    sudo apt update
    sudo apt install -y openssh-server
else
    echo "   ✅ SSH installato"
fi
echo

# 2. Avvia e abilita SSH
echo "2. Avvio servizio SSH..."
sudo systemctl start ssh
sudo systemctl enable ssh
sleep 2
echo

# 3. Verifica stato
echo "3. Verifica stato servizio..."
if sudo systemctl is-active --quiet ssh; then
    echo "   ✅ SSH è ATTIVO"
else
    echo "   ❌ SSH NON è attivo"
    echo "   Dettagli:"
    sudo systemctl status ssh --no-pager -l | head -10
fi
echo

# 4. Verifica porta
echo "4. Verifica porta 22..."
if sudo ss -tlnp | grep -q ":22"; then
    echo "   ✅ Porta 22 in ascolto"
    sudo ss -tlnp | grep ":22"
else
    echo "   ❌ Porta 22 NON in ascolto"
fi
echo

# 5. Configura firewall
echo "5. Configurazione firewall..."
if command -v ufw &> /dev/null; then
    sudo ufw allow 22/tcp 2>/dev/null
    sudo ufw reload 2>/dev/null
    echo "   ✅ Firewall configurato"
else
    echo "   ⚠️ UFW non installato (potrebbe non essere necessario)"
fi
echo

# 6. Verifica IP
echo "6. Verifica IP..."
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo "   IP attuale: $CURRENT_IP"
if [ "$CURRENT_IP" = "192.168.10.191" ]; then
    echo "   ✅ IP corretto (192.168.10.191)"
else
    echo "   ⚠️ IP diverso da 192.168.10.191"
    echo "   💡 Se l'IP è cambiato, aggiorna network_info.txt"
fi
echo

# 7. Test connessione locale
echo "7. Test connessione SSH locale..."
if timeout 2 ssh -o ConnectTimeout=1 -o StrictHostKeyChecking=no -o BatchMode=yes lab@localhost exit 2>/dev/null; then
    echo "   ✅ SSH funziona localmente"
else
    echo "   ⚠️ SSH non risponde localmente (potrebbe essere normale se richiede password)"
    echo "   💡 Prova manualmente: ssh lab@localhost"
fi
echo

# 8. Verifica configurazione
echo "8. Verifica configurazione SSH..."
if sudo sshd -t 2>&1 | grep -q "error"; then
    echo "   ❌ Errori nella configurazione:"
    sudo sshd -t
else
    echo "   ✅ Configurazione OK"
fi
echo

# 9. Mostra log recenti
echo "9. Log SSH recenti (ultimi 10 messaggi):"
sudo journalctl -u ssh -n 10 --no-pager | tail -5
echo

echo "================================================================================"
echo "✅ FIX COMPLETATO"
echo "================================================================================"
echo
echo "📍 PROSSIMI PASSI:"
echo "   1. Verifica connessione da remoto:"
echo "      ssh lab@192.168.10.191"
echo
echo "   2. Se non funziona, controlla:"
echo "      - IP corretto: hostname -I"
echo "      - Firewall: sudo ufw status"
echo "      - Log: sudo journalctl -u ssh -n 50"
echo





