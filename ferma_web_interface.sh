#!/bin/bash
# Script per fermare correttamente la web interface

echo "=================================================================================="
echo "🛑 FERMA WEB INTERFACE"
echo "=================================================================================="
echo ""

# Trova processi web_interface dell'utente corrente
PIDS=$(ps aux | grep -E 'web_interface|flask.*8080|python.*web_interface' | grep "^$(whoami)" | grep -v grep | awk '{print $2}')

if [ -z "$PIDS" ]; then
    echo "✅ Nessun processo web_interface trovato per l'utente corrente"
else
    echo "Trovati processi:"
    ps aux | grep -E 'web_interface|flask.*8080|python.*web_interface' | grep "^$(whoami)" | grep -v grep
    
    echo ""
    echo "Fermo processi..."
    for PID in $PIDS; do
        echo "  Killing PID $PID..."
        kill $PID 2>/dev/null || kill -9 $PID 2>/dev/null
    done
    
    sleep 2
    
    # Verifica che siano morti
    REMAINING=$(ps aux | grep -E 'web_interface|flask.*8080|python.*web_interface' | grep "^$(whoami)" | grep -v grep | awk '{print $2}')
    if [ -z "$REMAINING" ]; then
        echo "✅ Tutti i processi fermati"
    else
        echo "⚠️  Alcuni processi ancora attivi:"
        ps aux | grep -E 'web_interface|flask.*8080|python.*web_interface' | grep "^$(whoami)" | grep -v grep
        echo ""
        echo "Provo kill -9..."
        for PID in $REMAINING; do
            kill -9 $PID 2>/dev/null
        done
        sleep 1
    fi
fi

echo ""

# Verifica porta 8080
if netstat -tuln 2>/dev/null | grep -q ":8080 "; then
    echo "⚠️  Porta 8080 ancora in uso"
    echo "   Processi che usano porta 8080:"
    if command -v lsof > /dev/null 2>&1; then
        sudo lsof -i :8080 2>/dev/null || echo "   (richiede sudo per vedere)"
    else
        echo "   Installa lsof: sudo apt install lsof"
    fi
else
    echo "✅ Porta 8080 libera"
fi

echo ""
echo "=================================================================================="

