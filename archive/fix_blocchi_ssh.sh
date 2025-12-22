#!/bin/bash
# Script per trovare e fixare blocchi SSH

echo "================================================================================"
echo "DIAGNOSTICA E FIX BLOCCHI SSH"
echo "================================================================================"
echo

# 1. Verifica SSH è attivo
echo "1. VERIFICA SSH..."
if sudo systemctl is-active --quiet ssh; then
    echo "   ✅ SSH è ATTIVO (PID: $(pgrep -f 'sshd.*-D'))"
else
    echo "   ❌ SSH NON è attivo"
    exit 1
fi
echo

# 2. Verifica porta 22 in ascolto
echo "2. VERIFICA PORTA 22..."
if sudo ss -tlnp | grep -q ":22"; then
    echo "   ✅ Porta 22 in ascolto"
    echo "   Dettagli:"
    sudo ss -tlnp | grep ":22"
else
    echo "   ❌ Porta 22 NON in ascolto"
fi
echo

# 3. Verifica IP
echo "3. VERIFICA IP..."
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo "   IP: $CURRENT_IP"
if [ "$CURRENT_IP" != "192.168.10.191" ]; then
    echo "   ⚠️ IP diverso da 192.168.10.191"
fi
echo

# 4. Verifica UFW (firewall Ubuntu)
echo "4. VERIFICA FIREWALL UFW..."
if command -v ufw &> /dev/null; then
    UFW_STATUS=$(sudo ufw status | head -1)
    echo "   Stato: $UFW_STATUS"
    
    if echo "$UFW_STATUS" | grep -qi "active"; then
        echo "   ⚠️ UFW è ATTIVO"
        
        if sudo ufw status | grep -q "22/tcp.*ALLOW"; then
            echo "   ✅ SSH (22/tcp) è permesso"
        else
            echo "   ❌ SSH (22/tcp) NON è permesso - FIXO ORA!"
            sudo ufw allow 22/tcp
            sudo ufw reload
            echo "   ✅ SSH ora permesso"
        fi
    else
        echo "   ✅ UFW è INATTIVO (non blocca)"
    fi
else
    echo "   ⚠️ UFW non installato"
fi
echo

# 5. Verifica iptables (firewall Linux standard)
echo "5. VERIFICA IPTABLES..."
if sudo iptables -L -n | grep -q "DROP.*22\|REJECT.*22"; then
    echo "   ⚠️ IPTABLES blocca porta 22"
    echo "   Regole attuali:"
    sudo iptables -L -n | grep -A 5 -B 5 "22"
    echo ""
    echo "   💡 Per permettere SSH:"
    echo "      sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT"
    echo "      sudo iptables-save | sudo tee /etc/iptables/rules.v4"
else
    if sudo iptables -L INPUT -n | grep -q "ACCEPT.*22"; then
        echo "   ✅ IPTABLES permette porta 22"
    else
        echo "   ⚠️ Verifica regole IPTABLES:"
        sudo iptables -L INPUT -n | head -10
    fi
fi
echo

# 6. Verifica configurazione SSH (ListenAddress)
echo "6. VERIFICA CONFIGURAZIONE SSH..."
SSH_CONFIG="/etc/ssh/sshd_config"
if [ -f "$SSH_CONFIG" ]; then
    LISTEN_ADDRESS=$(sudo grep "^ListenAddress" "$SSH_CONFIG" | head -1)
    if [ -z "$LISTEN_ADDRESS" ]; then
        echo "   ✅ SSH ascolta su tutte le interfacce (default)"
    else
        echo "   Configurazione ListenAddress:"
        echo "   $LISTEN_ADDRESS"
        if echo "$LISTEN_ADDRESS" | grep -q "127.0.0.1"; then
            echo "   ❌ SSH ascolta solo su localhost - FIXO ORA!"
            sudo sed -i 's/^ListenAddress.*/#ListenAddress 0.0.0.0/' "$SSH_CONFIG"
            sudo systemctl restart ssh
            echo "   ✅ SSH ora ascolta su tutte le interfacce"
        fi
    fi
    
    # Verifica altre impostazioni importanti
    if sudo grep -q "^PermitRootLogin yes" "$SSH_CONFIG"; then
        echo "   ⚠️ Root login permesso (normale per lab user)"
    fi
    
    if sudo grep -q "^PasswordAuthentication no" "$SSH_CONFIG"; then
        echo "   ⚠️ Password authentication disabilitata"
        echo "   💡 Per abilitare: sudo sed -i 's/^PasswordAuthentication no/PasswordAuthentication yes/' $SSH_CONFIG"
    fi
else
    echo "   ❌ File configurazione SSH non trovato"
fi
echo

# 7. Verifica interfacce di rete
echo "7. VERIFICA INTERFACCE DI RETE..."
echo "   Interfacce attive:"
ip addr show | grep -E "^[0-9]+:|inet " | grep -B 1 "inet "
echo

# 8. Test connessione locale
echo "8. TEST CONNESSIONE LOCALE..."
if timeout 2 ssh -o ConnectTimeout=1 -o StrictHostKeyChecking=no -o BatchMode=yes lab@localhost exit 2>/dev/null; then
    echo "   ✅ SSH risponde localmente"
else
    echo "   ⚠️ SSH non risponde localmente"
fi
echo

# 9. Verifica log SSH per errori
echo "9. LOG SSH RECENTI (cerca errori)..."
sudo journalctl -u ssh -n 20 --no-pager | grep -i "error\|fail\|refus\|deny" | tail -5
if [ $? -ne 0 ]; then
    echo "   ✅ Nessun errore nei log recenti"
fi
echo

# 10. Riavvio SSH per applicare modifiche
echo "10. RIAVVIO SSH..."
sudo systemctl restart ssh
sleep 2
if sudo systemctl is-active --quiet ssh; then
    echo "   ✅ SSH riavviato correttamente"
else
    echo "   ❌ SSH non si è riavviato correttamente"
    sudo systemctl status ssh --no-pager | head -10
fi
echo

# 11. Verifica finale
echo "11. VERIFICA FINALE..."
echo "   Porta 22 in ascolto:"
sudo ss -tlnp | grep ":22"
echo
echo "   IP macchina:"
hostname -I
echo
echo "   Firewall UFW:"
sudo ufw status | head -3
echo

echo "================================================================================"
echo "✅ DIAGNOSTICA COMPLETATA"
echo "================================================================================"
echo
echo "📍 PROSSIMI PASSI:"
echo "   1. Prova a connetterti da remoto:"
echo "      ssh lab@$CURRENT_IP"
echo ""
echo "   2. Se ancora non funziona, verifica:"
echo "      - Router/switch non blocca porta 22"
echo "      - Nessun firewall hardware"
echo "      - Rete locale funziona (ping da altri dispositivi)"
echo ""











