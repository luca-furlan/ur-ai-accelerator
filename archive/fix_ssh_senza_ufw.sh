#!/bin/bash
# Fix SSH senza UFW (solo IPTABLES e configurazione SSH)

echo "================================================================================"
echo "FIX SSH - SENZA UFW (solo IPTABLES)"
echo "================================================================================"
echo

# 1. Verifica SSH è attivo
echo "1. VERIFICA SSH..."
if sudo systemctl is-active --quiet ssh; then
    echo "   ✅ SSH è ATTIVO"
    sudo systemctl status ssh --no-pager | head -3
else
    echo "   ❌ SSH NON è attivo"
    exit 1
fi
echo

# 2. Verifica IPTABLES
echo "2. VERIFICA IPTABLES..."
if command -v iptables &> /dev/null; then
    echo "   ✅ IPTABLES installato"
    
    # Mostra regole attuali per porta 22
    echo "   Regole attuali per porta 22:"
    sudo iptables -L INPUT -n | grep -A 2 -B 2 "22" || echo "      Nessuna regola specifica per porta 22"
    
    # Verifica se c'è una regola DROP/REJECT generale
    if sudo iptables -L INPUT -n | grep -q "DROP\|REJECT"; then
        echo "   ⚠️ Trovate regole DROP/REJECT - aggiungo eccezione per SSH"
        sudo iptables -I INPUT -p tcp --dport 22 -j ACCEPT
        echo "   ✅ Regola aggiunta: ACCEPT per porta 22"
    else
        # Aggiungi comunque la regola per sicurezza
        sudo iptables -I INPUT -p tcp --dport 22 -j ACCEPT
        echo "   ✅ Regola aggiunta: ACCEPT per porta 22"
    fi
    
    # Salva regole (se possibile)
    if [ -d /etc/iptables ]; then
        sudo iptables-save | sudo tee /etc/iptables/rules.v4 > /dev/null 2>&1
        echo "   ✅ Regole salvate"
    else
        echo "   💡 Per salvare regole permanentemente:"
        echo "      sudo mkdir -p /etc/iptables"
        echo "      sudo iptables-save | sudo tee /etc/iptables/rules.v4"
    fi
else
    echo "   ⚠️ IPTABLES non installato"
    echo "   💡 Installa: sudo apt install iptables"
fi
echo

# 3. Verifica configurazione SSH
echo "3. VERIFICA CONFIGURAZIONE SSH..."
SSH_CONFIG="/etc/ssh/sshd_config"

# Verifica ListenAddress
LISTEN_ADDR=$(sudo grep "^ListenAddress" "$SSH_CONFIG" 2>/dev/null | head -1)
if [ -z "$LISTEN_ADDR" ]; then
    echo "   ✅ SSH ascolta su tutte le interfacce (default)"
elif echo "$LISTEN_ADDR" | grep -q "127.0.0.1"; then
    echo "   ❌ SSH ascolta solo su localhost - FIXO ORA!"
    sudo sed -i 's/^ListenAddress.*/#ListenAddress 0.0.0.0/' "$SSH_CONFIG"
    echo "   ✅ SSH ora ascolta su tutte le interfacce"
else
    echo "   Configurazione ListenAddress: $LISTEN_ADDR"
fi

# Verifica PasswordAuthentication
if sudo grep -q "^PasswordAuthentication no" "$SSH_CONFIG"; then
    echo "   ❌ Password authentication disabilitata - FIXO ORA!"
    sudo sed -i 's/^PasswordAuthentication no/PasswordAuthentication yes/' "$SSH_CONFIG"
    echo "   ✅ Password authentication abilitata"
elif sudo grep -q "^PasswordAuthentication yes" "$SSH_CONFIG"; then
    echo "   ✅ Password authentication abilitata"
else
    echo "   ⚠️ PasswordAuthentication non specificata (default: yes)"
fi

# Verifica altre impostazioni importanti
if sudo grep -q "^PermitRootLogin prohibit-password" "$SSH_CONFIG"; then
    echo "   ⚠️ Root login solo con chiave (normale per lab user)"
fi
echo

# 4. Riavvia SSH
echo "4. RIAVVIO SSH..."
sudo systemctl restart ssh
sleep 2

if sudo systemctl is-active --quiet ssh; then
    echo "   ✅ SSH riavviato correttamente"
else
    echo "   ❌ SSH non si è riavviato correttamente"
    sudo systemctl status ssh --no-pager | head -10
fi
echo

# 5. Verifica finale
echo "5. VERIFICA FINALE..."
echo "   Porta 22 in ascolto:"
if sudo ss -tlnp | grep -q ":22"; then
    sudo ss -tlnp | grep ":22"
    echo "   ✅ Porta 22 è in ascolto"
    
    # Verifica che ascolti su 0.0.0.0 (non solo 127.0.0.1)
    if sudo ss -tlnp | grep ":22" | grep -q "0.0.0.0"; then
        echo "   ✅ SSH ascolta su tutte le interfacce (0.0.0.0)"
    else
        echo "   ⚠️ SSH potrebbe ascoltare solo su localhost"
    fi
else
    echo "   ❌ Porta 22 NON è in ascolto"
fi
echo

echo "   IP macchina:"
hostname -I
echo

echo "   Regole IPTABLES per porta 22:"
sudo iptables -L INPUT -n | grep "22" || echo "   Nessuna regola specifica"
echo

echo "================================================================================"
echo "✅ FIX COMPLETATO"
echo "================================================================================"
echo
echo "📍 PROSSIMI PASSI:"
echo "   1. Prova a connetterti da remoto:"
echo "      ssh lab@$(hostname -I | awk '{print $1}')"
echo ""
echo "   2. Se ancora non funziona, verifica:"
echo "      - Router/switch non blocca porta 22"
echo "      - Nessun altro firewall"
echo "      - Rete locale funziona"
echo ""











