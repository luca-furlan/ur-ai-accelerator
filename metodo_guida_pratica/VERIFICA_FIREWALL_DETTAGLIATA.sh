#!/bin/bash

# Script per verificare in dettaglio cosa blocca le porte

PC_IP="192.168.10.191"
PORTS=(50001 50002 50003 50004)

echo "=================================================================================="
echo "🔍 VERIFICA DETTAGLIATA FIREWALL E PORTE"
echo "=================================================================================="
echo ""

# Verifica UFW
echo "1. VERIFICA UFW"
echo "---------------"
if command -v ufw > /dev/null 2>&1; then
    echo "✅ UFW installato"
    sudo ufw status verbose
else
    echo "❌ UFW non installato"
    echo ""
    echo "Installazione UFW..."
    if command -v apt-get > /dev/null 2>&1; then
        sudo apt-get update
        sudo apt-get install -y ufw
        echo ""
        echo "Configurazione porte..."
        for PORT in "${PORTS[@]}"; do
            sudo ufw allow $PORT
        done
        echo "y" | sudo ufw enable
        echo "✅ UFW installato e configurato"
    else
        echo "⚠️  Sistema non basato su apt-get"
    fi
fi
echo ""

# Verifica iptables
echo "2. VERIFICA IPTABLES"
echo "---------------------"
if command -v iptables > /dev/null 2>&1; then
    echo "✅ iptables disponibile"
    echo ""
    echo "Regole INPUT:"
    sudo iptables -L INPUT -n -v | head -20
    echo ""
    echo "Verifica porte specifiche:"
    for PORT in "${PORTS[@]}"; do
        if sudo iptables -L INPUT -n | grep -q "$PORT"; then
            echo "✅ Porta $PORT trovata nelle regole"
        else
            echo "❌ Porta $PORT NON trovata nelle regole"
            echo "   Aggiungendo regola..."
            sudo iptables -A INPUT -p tcp --dport $PORT -j ACCEPT
            echo "✅ Regola aggiunta per porta $PORT"
        fi
    done
    echo ""
    echo "Salvataggio regole..."
    sudo iptables-save | sudo tee /etc/iptables/rules.v4 > /dev/null 2>&1 || \
    sudo sh -c "iptables-save > /etc/iptables/rules.v4" 2>/dev/null || \
    echo "⚠️  Impossibile salvare regole permanentemente"
else
    echo "❌ iptables non disponibile"
fi
echo ""

# Verifica firewalld
echo "3. VERIFICA FIREWALLD"
echo "---------------------"
if command -v firewall-cmd > /dev/null 2>&1; then
    if sudo firewall-cmd --state 2>/dev/null | grep -q "running"; then
        echo "✅ firewalld è attivo"
        echo ""
        echo "Porte aperte:"
        sudo firewall-cmd --list-ports
        echo ""
        echo "Aprendo porte necessarie..."
        for PORT in "${PORTS[@]}"; do
            sudo firewall-cmd --permanent --add-port=$PORT/tcp
        done
        sudo firewall-cmd --reload
        echo "✅ Porte configurate in firewalld"
    else
        echo "ℹ️  firewalld non attivo"
    fi
else
    echo "ℹ️  firewalld non installato"
fi
echo ""

# Verifica processi che ascoltano
echo "4. VERIFICA PROCESSI IN ASCOLTO"
echo "-------------------------------"
echo "Porte in ascolto:"
netstat -tuln | grep -E "5000[1-4]"
echo ""

# Verifica accessibilità porte
echo "5. TEST ACCESSIBILITÀ PORTE"
echo "----------------------------"
for PORT in "${PORTS[@]}"; do
    if timeout 1 bash -c "echo > /dev/tcp/$PC_IP/$PORT" 2>/dev/null; then
        echo "✅ Porta $PORT accessibile localmente"
    else
        echo "❌ Porta $PORT NON accessibile localmente"
    fi
done
echo ""

# Verifica SELinux (se presente)
echo "6. VERIFICA SELINUX"
echo "--------------------"
if command -v getenforce > /dev/null 2>&1; then
    SELINUX_STATUS=$(getenforce 2>/dev/null)
    echo "SELinux status: $SELINUX_STATUS"
    if [ "$SELINUX_STATUS" = "Enforcing" ]; then
        echo "⚠️  SELinux è in modalità Enforcing (potrebbe bloccare porte)"
        echo "   Verifica regole: sudo semanage port -l | grep 5000"
    fi
else
    echo "ℹ️  SELinux non presente"
fi
echo ""

# Riepilogo
echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "Per aprire le porte manualmente:"
echo ""
echo "UFW:"
echo "  sudo ufw allow 50001"
echo "  sudo ufw allow 50002"
echo "  sudo ufw allow 50003"
echo "  sudo ufw allow 50004"
echo ""
echo "iptables:"
echo "  sudo iptables -A INPUT -p tcp --dport 50001 -j ACCEPT"
echo "  sudo iptables -A INPUT -p tcp --dport 50002 -j ACCEPT"
echo "  sudo iptables -A INPUT -p tcp --dport 50003 -j ACCEPT"
echo "  sudo iptables -A INPUT -p tcp --dport 50004 -j ACCEPT"
echo "  sudo iptables-save"
echo ""
echo "firewalld:"
echo "  sudo firewall-cmd --permanent --add-port=50001/tcp"
echo "  sudo firewall-cmd --permanent --add-port=50002/tcp"
echo "  sudo firewall-cmd --permanent --add-port=50003/tcp"
echo "  sudo firewall-cmd --permanent --add-port=50004/tcp"
echo "  sudo firewall-cmd --reload"
echo ""








