#!/bin/bash
# Script per installare e configurare VNC Server sull'AI Accelerator

set -e

echo "================================================================================"
echo "INSTALLAZIONE VNC SERVER"
echo "================================================================================"
echo

# 1. Installazione TigerVNC
echo "1. Installazione TigerVNC Server..."
echo "--------------------------------------------------------------------------------"
sudo apt update
sudo apt install -y tigervnc-standalone-server tigervnc-common tigervnc-xorg-extension

echo
echo "2. Configurazione VNC..."
echo "--------------------------------------------------------------------------------"

# Crea directory VNC
mkdir -p ~/.vnc

# Crea file xstartup per desktop environment
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Avvia desktop environment (XFCE o GNOME)
if [ -f /etc/X11/xinit/xinitrc ]; then
    . /etc/X11/xinit/xinitrc
fi

# Se XFCE è disponibile, usalo
if command -v startxfce4 > /dev/null; then
    startxfce4 &
elif command -v gnome-session > /dev/null; then
    gnome-session &
else
    # Fallback: window manager minimale
    x-window-manager &
fi
EOF

chmod +x ~/.vnc/xstartup

echo "   ✅ File xstartup creato"

# 3. Imposta password VNC
echo
echo "3. Configurazione password VNC..."
echo "--------------------------------------------------------------------------------"
echo "   💡 Verrà richiesto di inserire una password VNC"
echo "   (usa 'easybot' o una password a tua scelta)"
echo

# Se la password non è già configurata, la chiediamo
if [ ! -f ~/.vnc/passwd ]; then
    echo "easybot" | vncpasswd -f > ~/.vnc/passwd 2>/dev/null || {
        echo "   ⚠️ Impossibile impostare password automaticamente"
        echo "   Esegui manualmente: vncpasswd"
    }
    chmod 600 ~/.vnc/passwd
    echo "   ✅ Password VNC configurata (default: easybot)"
else
    echo "   ✅ Password VNC già configurata"
fi

# 4. Installazione desktop environment (XFCE - leggero)
echo
echo "4. Installazione Desktop Environment (XFCE)..."
echo "--------------------------------------------------------------------------------"
sudo apt install -y xfce4 xfce4-goodies || {
    echo "   ⚠️ Errore installazione XFCE, provo con desktop minimale"
    sudo apt install -y xfce4-core || true
}

# Aggiorna xstartup per XFCE
if command -v startxfce4 > /dev/null; then
    cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
/etc/X11/xinit/xinitrc
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
x-window-manager &
startxfce4 &
EOF
    chmod +x ~/.vnc/xstartup
    echo "   ✅ XFCE configurato"
fi

# 5. Avvia VNC server
echo
echo "5. Avvio VNC Server..."
echo "--------------------------------------------------------------------------------"

# Kill eventuali sessioni esistenti
vncserver -kill :1 2>/dev/null || true

# Avvia nuova sessione
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no

echo
echo "================================================================================"
echo "✅ VNC SERVER INSTALLATO E AVVIATO"
echo "================================================================================"
echo
echo "📍 INFORMAZIONI CONNESSIONE:"
echo "   IP: $(hostname -I | awk '{print $1}')"
echo "   Porta: 5901"
echo "   Display: :1"
echo "   Risoluzione: 1920x1080"
echo
echo "🔌 CONNESSIONE:"
echo "   Client VNC → $(hostname -I | awk '{print $1}'):5901"
echo "   Password: easybot (o quella configurata)"
echo
echo "💡 COMANDI UTILI:"
echo "   Avvia VNC:    vncserver :1 -geometry 1920x1080"
echo "   Ferma VNC:    vncserver -kill :1"
echo "   Lista sessioni: vncserver -list"
echo
echo "⚠️ NOTA: Se la risoluzione è troppo alta, prova:"
echo "   vncserver :1 -geometry 1280x720"
echo












