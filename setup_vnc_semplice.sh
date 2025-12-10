#!/bin/bash
# Setup VNC con desktop funzionante

echo "================================================================================"
echo "SETUP VNC SEMPLICE E FUNZIONANTE"
echo "================================================================================"
echo

# Kill sessioni esistenti
vncserver -kill :1 2>/dev/null || true
vncserver -kill :2 2>/dev/null || true
sleep 2

echo "1. Test con xterm (desktop minimale)..."
echo "--------------------------------------------------------------------------------"

# Prova con xterm semplice
vncserver :1 -geometry 1280x720 -depth 24 -localhost no -xstartup /usr/bin/xterm

sleep 3

# Verifica se funziona
if vncserver -list | grep -q ":1"; then
    echo "   ✅ VNC avviato con xterm"
    echo "   💡 Connettiti ora per testare: 192.168.10.191:5901"
    echo
    echo "   Premi INVIO quando hai testato la connessione..."
    read
    
    # Kill per configurare desktop completo
    vncserver -kill :1
else
    echo "   ❌ VNC non si avvia"
    exit 1
fi

echo
echo "2. Installazione dipendenze XFCE complete..."
echo "--------------------------------------------------------------------------------"
sudo apt install -y xfce4 xfce4-goodies dbus-x11

echo
echo "3. Configurazione xstartup per XFCE..."
echo "--------------------------------------------------------------------------------"

cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
export XKL_XMODMAP_DISABLE=1
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Avvia DBUS
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval $(dbus-launch --sh-syntax)
    export DBUS_SESSION_BUS_ADDRESS
fi

[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources

xsetroot -solid grey
vncconfig -iconic &

# Variabili XFCE
export XDG_CURRENT_DESKTOP="XFCE"
export XDG_MENU_PREFIX="xfce-"
export XDG_SESSION_DESKTOP="xfce"
export XDG_SESSION_TYPE=x11

# Avvia XFCE
startxfce4 &
EOF

chmod +x ~/.vnc/xstartup

echo "   ✅ xstartup configurato"

echo
echo "4. Avvio VNC con XFCE..."
echo "--------------------------------------------------------------------------------"
vncserver :1 -geometry 1280x720 -depth 24 -localhost no

sleep 3

if vncserver -list | grep -q ":1"; then
    echo "   ✅ VNC avviato con XFCE"
else
    echo "   ⚠️ XFCE non si avvia, uso desktop minimale..."
    
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
    chmod +x ~/.vnc/xstartup
    vncserver -kill :1
    vncserver :1 -geometry 1280x720 -depth 24 -localhost no
fi

echo
echo "================================================================================"
echo "✅ VNC CONFIGURATO"
echo "================================================================================"
echo
echo "📍 CONNESSIONE:"
echo "   IP: 192.168.10.191:5901"
echo "   Password: easybot"
echo
echo "💡 Client VNC:"
echo "   - TightVNC Viewer: https://www.tightvnc.com/download.php"
echo "   - RealVNC Viewer: https://www.realvnc.com/download/viewer/"
echo
vncserver -list
echo






