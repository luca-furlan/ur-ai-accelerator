#!/bin/bash
# Fix configurazione VNC xstartup

echo "================================================================================"
echo "FIX CONFIGURAZIONE VNC XSTARTUP"
echo "================================================================================"
echo

# Kill sessioni esistenti
echo "1. Pulizia sessioni VNC esistenti..."
vncserver -kill :1 2>/dev/null || true
vncserver -kill :2 2>/dev/null || true
sleep 2

# Verifica se XFCE è installato
echo
echo "2. Verifica Desktop Environment..."
if command -v startxfce4 > /dev/null; then
    echo "   ✅ XFCE installato"
    DESKTOP="xfce4"
elif command -v gnome-session > /dev/null; then
    echo "   ✅ GNOME disponibile"
    DESKTOP="gnome"
else
    echo "   ⚠️ Desktop environment non trovato, installo XFCE..."
    sudo apt install -y xfce4 xfce4-goodies || {
        echo "   ❌ Errore installazione XFCE"
        DESKTOP="minimal"
    }
fi

# Crea xstartup corretto
echo
echo "3. Creazione xstartup corretto..."
mkdir -p ~/.vnc

if [ "$DESKTOP" = "xfce4" ]; then
    cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
export XKL_XMODMAP_DISABLE=1
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources

xsetroot -solid grey
vncconfig -iconic &

# Avvia XFCE
export XDG_CURRENT_DESKTOP="XFCE"
export XDG_MENU_PREFIX="xfce-"
export XDG_SESSION_DESKTOP="xfce"

startxfce4 &
EOF
elif [ "$DESKTOP" = "gnome" ]; then
    cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
xsetroot -solid grey
vncconfig -iconic &
gnome-session &
EOF
else
    # Fallback minimale
    cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
xsetroot -solid grey
vncconfig -iconic &
xterm -geometry 80x24+10+10 -ls -title "$VNCDESKTOP Desktop" &
twm &
EOF
fi

chmod +x ~/.vnc/xstartup
echo "   ✅ xstartup creato"

# Verifica dipendenze
echo
echo "4. Verifica dipendenze..."
if ! command -v startxfce4 > /dev/null && [ "$DESKTOP" = "xfce4" ]; then
    echo "   ⚠️ XFCE non trovato, installo..."
    sudo apt install -y xfce4 xfce4-goodies
fi

# Avvia VNC
echo
echo "5. Avvio VNC Server..."
vncserver -kill :1 2>/dev/null || true
sleep 1

# Prova con risoluzione standard
vncserver :1 -geometry 1280x720 -depth 24 -localhost no

echo
echo "================================================================================"
echo "✅ VNC CONFIGURATO"
echo "================================================================================"
echo
echo "📍 Connessione:"
echo "   IP: $(hostname -I | awk '{print $1}'):5901"
echo "   Password: easybot"
echo
echo "💡 Se ancora non funziona, prova:"
echo "   vncserver -kill :1"
echo "   vncserver :1 -xstartup /usr/bin/xterm"
echo






