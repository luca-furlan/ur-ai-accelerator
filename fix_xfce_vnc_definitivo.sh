#!/bin/bash
# Fix definitivo per XFCE su VNC

echo "=== FIX XFCE VNC DEFINITIVO ==="

# Kill sessioni esistenti
vncserver -kill :2 2>/dev/null || true
sleep 2

# Crea xstartup che mantiene il processo attivo
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

# Avvia XFCE in background e mantieni lo script attivo
if command -v startxfce4 &> /dev/null; then
    startxfce4 &
    # Mantieni lo script attivo
    wait
else
    echo "startxfce4 non trovato!" > ~/.vnc/xfce_error.log
    xterm -geometry 80x24+10+10 -ls -title "VNC Desktop" &
    wait
fi
EOF

chmod +x ~/.vnc/xstartup

# Prova un approccio alternativo: usa /etc/X11/Xsession
echo "=== PROVA CON XSESSION ==="
cat > ~/.vnc/xstartup.xsession << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
xsetroot -solid grey
vncconfig -iconic &
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP="XFCE"
export XDG_SESSION_DESKTOP="xfce"
exec /etc/X11/Xsession xfce4-session
EOF

chmod +x ~/.vnc/xstartup.xsession

# Prova anche con startxfce4 diretto
echo "=== PROVA CON STARTXFCE4 DIRETTO ==="
cat > ~/.vnc/xstartup.direct << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
xsetroot -solid grey
vncconfig -iconic &
export XDG_SESSION_TYPE=x11
export XDG_CURRENT_DESKTOP="XFCE"
export XDG_SESSION_DESKTOP="xfce"
export XDG_MENU_PREFIX="xfce-"
/usr/bin/startxfce4
EOF

chmod +x ~/.vnc/xstartup.direct

echo "✅ Creati 3 versioni di xstartup:"
echo "   1. ~/.vnc/xstartup (standard)"
echo "   2. ~/.vnc/xstartup.xsession (con Xsession)"
echo "   3. ~/.vnc/xstartup.direct (startxfce4 diretto)"
echo
echo "Prova in ordine:"
echo "1. cp ~/.vnc/xstartup.xsession ~/.vnc/xstartup"
echo "2. vncserver :2 -geometry 1280x720 -depth 24 -localhost no"
echo
echo "Se non funziona, prova:"
echo "1. cp ~/.vnc/xstartup.direct ~/.vnc/xstartup"
echo "2. vncserver :2 -geometry 1280x720 -depth 24 -localhost no"





