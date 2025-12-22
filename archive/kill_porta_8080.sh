#!/bin/bash
# Script per liberare porta 8080 automaticamente

PORT=${1:-8080}

echo "→ Liberazione porta $PORT..."

# Trova processo
PID=$(lsof -ti :$PORT 2>/dev/null | head -1)

if [ -z "$PID" ]; then
    echo "✅ Porta $PORT già libera"
    exit 0
fi

echo "⚠️  Porta $PORT occupata da processo $PID"
echo "→ Terminazione..."

# Kill graceful
kill -TERM $PID 2>/dev/null || true
sleep 2

# Kill forzato se necessario
if kill -0 $PID 2>/dev/null; then
    echo "→ Kill forzato..."
    kill -9 $PID 2>/dev/null || true
    sleep 1
fi

# Verifica
if lsof -ti :$PORT >/dev/null 2>&1; then
    echo "❌ Impossibile liberare porta $PORT"
    exit 1
else
    echo "✅ Porta $PORT liberata"
    exit 0
fi




