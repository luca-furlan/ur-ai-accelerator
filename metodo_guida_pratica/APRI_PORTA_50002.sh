#!/bin/bash

# Script per aprire porta 50002 nel firewall (se necessario)

echo "=================================================================================="
echo "🔧 APERTURA PORTA 50002 NEL FIREWALL"
echo "=================================================================================="
echo ""

PORT="50002"

# Verifica se ufw è installato
if command -v ufw > /dev/null 2>&1; then
    echo "✅ UFW trovato, apro porta..."
    sudo ufw allow $PORT
    sudo ufw reload
    echo "✅ Porta $PORT aperta con UFW"
else
    echo "⚠️  UFW non installato, verifico iptables..."
    
    # Verifica se iptables è disponibile
    if command -v iptables > /dev/null 2>&1; then
        echo "✅ IPTABLES trovato"
        
        # Verifica se porta è già aperta
        if sudo iptables -L -n | grep -q "$PORT"; then
            echo "✅ Porta $PORT già aperta in iptables"
        else
            echo "⚠️  Porta $PORT non trovata nelle regole iptables"
            echo "   Nota: Se non c'è firewall attivo, la porta è già accessibile"
        fi
        
        # Mostra regole correnti
        echo ""
        echo "Regole iptables correnti (prime 10 righe):"
        sudo iptables -L -n | head -10
    else
        echo "⚠️  Né UFW né IPTABLES trovati"
        echo "   Probabilmente non c'è firewall attivo"
    fi
fi

echo ""
echo "=================================================================================="
echo "🔍 VERIFICA MACCHINA IN ASCOLTO"
echo "=================================================================================="
echo ""

if netstat -tuln | grep -q ":$PORT "; then
    echo "✅ Macchina remota IN ASCOLTO sulla porta $PORT"
    netstat -tuln | grep ":$PORT "
else
    echo "❌ Macchina remota NON in ascolto sulla porta $PORT"
    echo ""
    echo "SOLUZIONE: Avvia driver ROS2 PRIMA!"
    echo "cd ~/MekoAiAccelerator/metodo_guida_pratica"
    echo "./START_RAPIDO.sh"
fi

echo ""
echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "1. Firewall: Verificato (probabilmente non attivo o già configurato)"
echo "2. Porta $PORT: Verifica se macchina è in ascolto sopra"
echo ""
echo "IMPORTANTE:"
echo "- Se macchina NON è in ascolto, avvia driver ROS2 PRIMA"
echo "- Il driver ROS2 mette automaticamente la macchina in ascolto sulla 50002"
echo ""
echo "=================================================================================="








