#!/bin/bash
# Script per killare forzatamente il web interface

echo "🔍 Cercando processi web interface..."

# Trova tutti i processi
PIDS=$(pgrep -f "python3 -m remote_ur_control.web_interface" || pgrep -f "web_interface.py" || pgrep -f "flask.*8080")

if [ -z "$PIDS" ]; then
    echo "✅ Nessun processo web interface trovato"
    exit 0
fi

echo "📋 Processi trovati:"
ps aux | grep -E "python3.*web_interface|flask.*8080" | grep -v grep

echo ""
echo "🛑 Terminando processi..."

for PID in $PIDS; do
    echo "   Killando PID $PID..."
    kill -9 $PID 2>/dev/null || true
done

sleep 2

# Verifica che siano morti
REMAINING=$(pgrep -f "python3 -m remote_ur_control.web_interface" || pgrep -f "web_interface.py" || pgrep -f "flask.*8080")

if [ -z "$REMAINING" ]; then
    echo "✅ Tutti i processi terminati"
else
    echo "⚠️  Alcuni processi potrebbero essere ancora attivi:"
    ps aux | grep -E "python3.*web_interface|flask.*8080" | grep -v grep
fi







