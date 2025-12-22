#!/bin/bash
# Script completo per fixare VNC sulla macchina AI Accelerator

echo "================================================================================"
echo "FIX VNC COMPLETO - AI ACCELERATOR"
echo "================================================================================"
echo

# 1. Verifica se VNC è installato
echo "1. VERIFICA INSTALLAZIONE VNC..."
if command -v vncserver &> /dev/null; then
    echo "   ✅ VNC installato"
    vncserver -version 2>/dev/null | head -1 || echo "   (versione non disponibile)"
else
    echo "   ❌ VNC non installato"
    echo "   💡 Installa: sudo apt install -y tigervnc-standalone-server tigervnc-common"
    exit 1
fi
echo

# 2. Kill sessioni VNC esistenti
echo "2. PULIZIA SESSIONI VNC ESISTENTI..."
vncserver -kill :1 2>/dev/null || true
vncserver -kill :2 2>/dev/null || true
sleep 2
echo "   ✅ Sessioni VNC terminate"
echo

# 3. Verifica desktop environment
echo "3. VERIFICA DESKTOP ENVIRONMENT..."
if command -v startxfce4 &> /dev/null; then
    echo "   ✅ XFCE installato"
    DESKTOP="xfce4"
elif command -v startlxde &> /dev/null; then
    echo "   ✅ LXDE installato"
    DESKTOP="lxde"
else
    echo "   ⚠️ Nessun desktop environment completo trovato"
    echo "   💡 Installa: sudo apt install -y xfce4 xfce4-goodies"
    DESKTOP="minimal"
fi
echo

# 4. Configura xstartup
echo "4. CONFIGURAZIONE XSTARTUP..."
mkdir -p ~/.vnc

if [ "$DESKTOP" = "xfce4" ]; then
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

export XDG_CURRENT_DESKTOP="XFCE"
export XDG_MENU_PREFIX="xfce-"
export XDG_SESSION_DESKTOP="xfce"
export XDG_SESSION_TYPE=x11

startxfce4 &
EOF
elif [ "$DESKTOP" = "lxde" ]; then
    cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
xsetroot -solid grey
vncconfig -iconic &
startlxde &
EOF
else
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
echo "   ✅ xstartup configurato per $DESKTOP"
echo

# 5. Verifica password VNC
echo "5. VERIFICA PASSWORD VNC..."
if [ ! -f ~/.vnc/passwd ]; then
    echo "   ⚠️ Password VNC non configurata"
    echo "   💡 Configura con: vncpasswd"
    echo "   (Password suggerita: easybot)"
else
    echo "   ✅ Password VNC configurata"
fi
echo

# 6. Permetti VNC nel firewall (IPTABLES)
echo "6. CONFIGURAZIONE FIREWALL..."
if sudo iptables -L INPUT -n | grep -q "5901"; then
    echo "   ✅ Porta 5901 già permessa in IPTABLES"
else
    sudo iptables -I INPUT -p tcp --dport 5901 -j ACCEPT
    echo "   ✅ Porta 5901 permessa in IPTABLES"
fi
echo

# 7. Avvia VNC Server
echo "7. AVVIO VNC SERVER..."
vncserver :1 -geometry 1280x720 -depth 24 -localhost no
sleep 3
echo

# 8. Verifica VNC è attivo
echo "8. VERIFICA VNC..."
if ps aux | grep -q "[X]tigervnc.*:1" || ps aux | grep -q "[X]vnc.*:1"; then
    echo "   ✅ VNC Server è ATTIVO"
    ps aux | grep -E "[X]tigervnc|[X]vnc" | grep -v grep | head -2
else
    echo "   ❌ VNC Server NON è attivo"
    echo "   💡 Controlla log: tail -50 ~/.vnc/*:1.log"
fi
echo

# 9. Verifica porta 5901
echo "9. VERIFICA PORTA 5901..."
if ss -tlnp | grep -q ":5901"; then
    echo "   ✅ Porta 5901 in ascolto"
    ss -tlnp | grep ":5901"
else
    echo "   ❌ Porta 5901 NON in ascolto"
fi
echo

# 10. Mostra sessioni VNC
echo "10. SESSIONI VNC ATTIVE..."
vncserver -list
echo

echo "================================================================================"
echo "✅ FIX VNC COMPLETATO"
echo "================================================================================"
echo
echo "📍 CONNESSIONE VNC:"
echo "   IP: 192.168.10.191:5901"
echo "   Password: easybot"
echo
echo "💡 CLIENT VNC:"
echo "   - TightVNC Viewer: https://www.tightvnc.com/download.php"
echo "   - RealVNC Viewer: https://www.realvnc.com/download/viewer/"
echo











