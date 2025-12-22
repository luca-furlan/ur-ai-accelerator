#!/bin/bash
# Script rapido per verificare SSH sulla macchina AI Accelerator

echo "================================================================================"
echo "VERIFICA SSH - AI ACCELERATOR"
echo "================================================================================"
echo

echo "1. STATO SERVIZIO SSH"
echo "--------------------------------------------------------------------------------"
if sudo systemctl is-active --quiet ssh; then
    echo "   ✅ SSH è ATTIVO"
    sudo systemctl status ssh --no-pager | head -5
else
    echo "   ❌ SSH NON è attivo"
    sudo systemctl status ssh --no-pager | head -5
fi
echo

echo "2. PORTA 22 IN ASCOLTO"
echo "--------------------------------------------------------------------------------"
if sudo ss -tlnp | grep -q ":22"; then
    echo "   ✅ Porta 22 è in ascolto"
    sudo ss -tlnp | grep ":22"
else
    echo "   ❌ Porta 22 NON è in ascolto"
fi
echo

echo "3. PROCESSI SSH"
echo "--------------------------------------------------------------------------------"
SSH_PROCESSES=$(ps aux | grep sshd | grep -v grep)
if [ -n "$SSH_PROCESSES" ]; then
    echo "   ✅ Processi SSH trovati:"
    echo "$SSH_PROCESSES" | while read line; do
        echo "      $line"
    done
else
    echo "   ❌ Nessun processo SSH trovato"
fi
echo

echo "4. IP MACCHINA"
echo "--------------------------------------------------------------------------------"
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo "   IP attuale: $CURRENT_IP"
if [ "$CURRENT_IP" = "192.168.10.191" ]; then
    echo "   ✅ IP corretto (192.168.10.191)"
else
    echo "   ⚠️ IP diverso da 192.168.10.191"
    echo "   💡 Questo potrebbe essere il problema!"
fi
echo

echo "5. FIREWALL"
echo "--------------------------------------------------------------------------------"
if command -v ufw &> /dev/null; then
    UFW_STATUS=$(sudo ufw status | head -1)
    echo "   Stato: $UFW_STATUS"
    if sudo ufw status | grep -q "22/tcp.*ALLOW"; then
        echo "   ✅ SSH (22/tcp) è permesso"
    else
        echo "   ⚠️ SSH (22/tcp) potrebbe non essere permesso"
        echo "   💡 Esegui: sudo ufw allow 22/tcp"
    fi
else
    echo "   ⚠️ UFW non installato (potrebbe non essere necessario)"
fi
echo

echo "6. TEST CONNESSIONE LOCALE"
echo "--------------------------------------------------------------------------------"
if timeout 2 ssh -o ConnectTimeout=1 -o StrictHostKeyChecking=no -o BatchMode=yes lab@localhost exit 2>/dev/null; then
    echo "   ✅ SSH risponde localmente"
else
    echo "   ⚠️ SSH non risponde localmente"
    echo "   💡 Prova manualmente: ssh lab@localhost"
fi
echo

echo "7. LOG SSH RECENTI"
echo "--------------------------------------------------------------------------------"
sudo journalctl -u ssh -n 5 --no-pager | tail -3
echo

echo "================================================================================"
echo "RIEPILOGO"
echo "================================================================================"
if sudo systemctl is-active --quiet ssh && sudo ss -tlnp | grep -q ":22"; then
    echo "✅ SSH è ATTIVO e FUNZIONANTE"
    echo ""
    echo "📍 Connettiti con:"
    echo "   ssh lab@$CURRENT_IP"
    echo "   Password: easybot"
else
    echo "❌ SSH NON è attivo o non funziona"
    echo ""
    echo "💡 Per avviare SSH:"
    echo "   sudo systemctl start ssh"
    echo "   sudo systemctl enable ssh"
    echo "   sudo ufw allow 22/tcp"
fi
echo











