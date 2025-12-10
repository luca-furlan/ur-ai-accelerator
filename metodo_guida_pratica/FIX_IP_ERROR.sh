#!/bin/bash

echo "=================================================================================="
echo "🔧 FIX IP ERROR - Verifica configurazione"
echo "=================================================================================="
echo ""

ROBOT_IP="192.168.10.194"
REMOTE_IP="192.168.10.191"
PORT="50002"

echo "📡 Verifica IP macchina remota..."
LOCAL_IP=$(hostname -I | grep -o '192.168.10.[0-9]*' | head -1)

if [ "$LOCAL_IP" == "$REMOTE_IP" ]; then
    echo "✅ IP macchina remota corretto: $REMOTE_IP"
else
    echo "⚠️  IP macchina remota: $LOCAL_IP"
    echo "   Dovrebbe essere: $REMOTE_IP"
    echo "   Aggiorna IP Host sul Teach Pendant con: $LOCAL_IP"
fi

echo ""
echo "🔌 Verifica porta $PORT..."
if timeout 2 bash -c "</dev/tcp/$ROBOT_IP/$PORT" 2>/dev/null; then
    echo "✅ Porta $PORT raggiungibile!"
else
    echo "❌ Porta $PORT NON raggiungibile"
    echo ""
    echo "VERIFICA SUL TEACH PENDANT:"
    echo "1. IP Host deve essere: $REMOTE_IP"
    echo "2. Porta deve essere: $PORT"
    echo "3. IP e Porta devono essere SEPARATI"
    echo "4. IP deve avere PUNTI (.) non SPAZI"
    echo "5. Programma deve essere in PLAYING"
    echo ""
    echo "Configurazione CORRETTA:"
    echo "   IP Host: $REMOTE_IP"
    echo "   Porta: $PORT"
    echo ""
    echo "Configurazione SBAGLIATA (quello che vedi):"
    echo "   IP Host: 192 168 561 50002"
    echo "   (spazi, numero sbagliato, porta insieme)"
fi

echo ""
echo "=================================================================================="

