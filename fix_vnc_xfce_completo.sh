#!/bin/bash
# Fix VNC per avviare XFCE desktop completo

echo "================================================================================"
echo "CONFIGURAZIONE VNC CON XFCE DESKTOP COMPLETO"
echo "================================================================================"
echo

# 1. Verifica XFCE installato
echo "1. VERIFICA XFCE..."
if command -v startxfce4 &> /dev/null; then
    echo "   ✅ XFCE installato"
else
    echo "   ❌ XFCE non installato"
    echo "   💡 Installa: sudo apt install -y xfce4 xfce4-goodies dbus-x11"
    exit 1
fi
echo

# 2. Verifica dbus-x11
echo "2. VERIFICA DBUS..."
if command -v dbus-launch &> /dev/null; then
    echo "   ✅ dbus-launch disponibile"
else
    echo "   ⚠️ dbus-launch non trovato"
    echo "   💡 Installa: sudo apt install -y dbus-x11"
fi
echo

# 3. Kill sessioni VNC esistenti
echo "3. PULIZIA SESSIONI VNC..."
vncserver -kill :2 2>/dev/null || true
sleep 2
echo "   ✅ Sessioni terminate"
echo

# 4. Crea xstartup corretto per XFCE
echo "4. CONFIGURAZIONE XSTARTUP PER XFCE..."
mkdir -p ~/.vnc

cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
export XKL_XMODMAP_DISABLE=1
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Avvia DBUS
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    if command -v dbus-launch &> /dev/null; then
        eval $(dbus-launch --sh-syntax --exit-with-session)
        export DBUS_SESSION_BUS_ADDRESS
    fi
fi

# Carica risorse X
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources

# Imposta sfondo
xsetroot -solid grey

# Avvia vncconfig
vncconfig -iconic &

# Variabili ambiente XFCE
export XDG_CURRENT_DESKTOP="XFCE"
export XDG_MENU_PREFIX="xfce-"
export XDG_SESSION_DESKTOP="xfce"
export XDG_SESSION_TYPE=x11

# Avvia XFCE
if command -v startxfce4 &> /dev/null; then
    startxfce4 &
else
    echo "startxfce4 non trovato!" > ~/.vnc/xfce_error.log
    xterm -geometry 80x24+10+10 -ls -title "VNC Desktop" &
fi
EOF

chmod +x ~/.vnc/xstartup
echo "   ✅ xstartup configurato"
echo

# 5. Verifica permessi
echo "5. VERIFICA PERMESSI..."
ls -la ~/.vnc/xstartup
echo

# 6. Avvia VNC con XFCE
echo "6. AVVIO VNC CON XFCE..."
vncserver :2 -geometry 1280x720 -depth 24 -localhost no
sleep 5
echo

# 7. Verifica VNC è attivo
echo "7. VERIFICA VNC..."
if ps aux | grep -q "[X]tigervnc.*:2"; then
    echo "   ✅ VNC Server ATTIVO"
    ps aux | grep "[X]tigervnc.*:2" | grep -v grep | head -1
else
    echo "   ❌ VNC Server NON attivo"
    echo "   💡 Controlla log: tail -50 ~/.vnc/*:2.log"
fi
echo

# 8. Verifica porta
echo "8. VERIFICA PORTA 5902..."
if ss -tlnp | grep -q ":5902"; then
    echo "   ✅ Porta 5902 in ascolto"
    ss -tlnp | grep ":5902"
else
    echo "   ❌ Porta 5902 NON in ascolto"
fi
echo

# 9. Mostra log se ci sono errori
echo "9. LOG VNC (ultimi 20 righe)..."
if [ -f ~/.vnc/*:2.log ]; then
    tail -20 ~/.vnc/*:2.log 2>/dev/null | tail -10
else
    echo "   Log non disponibile"
fi
echo

echo "================================================================================"
echo "✅ CONFIGURAZIONE COMPLETATA"
echo "================================================================================"
echo
echo "📍 CONNESSIONE:"
echo "   IP: 192.168.10.191:5902"
echo "   Password: easybot"
echo
echo "💡 Se vedi ancora solo xterm, controlla:"
echo "   tail -50 ~/.vnc/*:2.log"
echo "   cat ~/.vnc/xstartup"
echo





