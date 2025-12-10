#!/bin/bash
# Pulizia completa e riavvio VNC

echo "=== PULIZIA E RIAVVIO VNC ==="

# Kill tutte le sessioni VNC
echo "1. Kill sessioni VNC esistenti..."
vncserver -kill :1 2>/dev/null || true
vncserver -kill :2 2>/dev/null || true
pkill -9 Xtigervnc 2>/dev/null || true
pkill -9 Xvnc 2>/dev/null || true
sleep 3
echo "   ✅ Sessioni terminate"
echo

# Verifica che non ci siano processi rimasti
echo "2. Verifica processi VNC..."
if ps aux | grep -E "[X]tigervnc|[X]vnc" | grep -v grep; then
    echo "   ⚠️ Processi VNC ancora attivi, kill forzato..."
    pkill -9 -f "vnc\|Xvnc\|Xtigervnc" 2>/dev/null || true
    sleep 2
else
    echo "   ✅ Nessun processo VNC attivo"
fi
echo

# Permetti porta 5901 in IPTABLES (se possibile senza sudo)
echo "3. Configurazione firewall..."
echo "   💡 Se necessario, esegui manualmente:"
echo "      sudo iptables -I INPUT -p tcp --dport 5901 -j ACCEPT"
echo

# Avvia VNC
echo "4. Avvio VNC Server..."
vncserver :1 -geometry 1280x720 -depth 24 -localhost no
sleep 5
echo

# Verifica
echo "5. Verifica VNC..."
if ps aux | grep -q "[X]tigervnc.*:1"; then
    echo "   ✅ VNC Server ATTIVO"
    ps aux | grep "[X]tigervnc.*:1" | grep -v grep
else
    echo "   ❌ VNC Server NON attivo"
    echo "   💡 Controlla log:"
    echo "      tail -50 ~/.vnc/*:1.log"
fi
echo

echo "6. Porta 5901..."
if ss -tlnp | grep -q ":5901"; then
    echo "   ✅ Porta 5901 in ascolto"
    ss -tlnp | grep ":5901"
else
    echo "   ❌ Porta 5901 NON in ascolto"
fi
echo

echo "7. Sessioni VNC:"
vncserver -list
echo

echo "=== COMPLETATO ==="
echo "Connettiti a: 192.168.10.191:5901"
echo "Password: easybot"





