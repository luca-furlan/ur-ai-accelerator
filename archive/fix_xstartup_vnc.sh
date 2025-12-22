#!/bin/bash
# Fix xstartup per VNC

echo "=== FIX XSTARTUP VNC ==="

# Crea xstartup semplice e funzionante
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Avvia DBUS se disponibile
if command -v dbus-launch &> /dev/null; then
    if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
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

# Prova XFCE
if command -v startxfce4 &> /dev/null; then
    export XDG_CURRENT_DESKTOP="XFCE"
    export XDG_MENU_PREFIX="xfce-"
    export XDG_SESSION_DESKTOP="xfce"
    export XDG_SESSION_TYPE=x11
    startxfce4 &
# Altrimenti prova LXDE
elif command -v startlxde &> /dev/null; then
    startlxde &
# Altrimenti desktop minimale
else
    xterm -geometry 80x24+10+10 -ls -title "$VNCDESKTOP Desktop" &
    twm &
fi
EOF

chmod +x ~/.vnc/xstartup
echo "✅ xstartup aggiornato"
echo

# Verifica dipendenze
echo "=== VERIFICA DIPENDENZE ==="
if ! command -v dbus-launch &> /dev/null; then
    echo "⚠️ dbus-launch non trovato"
    echo "💡 Installa: sudo apt install -y dbus-x11"
fi

if ! command -v startxfce4 &> /dev/null; then
    echo "⚠️ startxfce4 non trovato"
    echo "💡 Installa: sudo apt install -y xfce4 xfce4-goodies"
fi

if ! command -v xterm &> /dev/null; then
    echo "⚠️ xterm non trovato"
    echo "💡 Installa: sudo apt install -y xterm"
fi
echo

echo "=== PROVA VNC ==="
echo "Esegui: vncserver :2 -geometry 1280x720 -depth 24 -localhost no"











