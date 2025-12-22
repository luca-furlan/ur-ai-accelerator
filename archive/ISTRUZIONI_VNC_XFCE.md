# ✅ VNC CON XFCE - ISTRUZIONI

## 📍 CONNESSIONE

- **IP:** 192.168.10.191
- **Porta:** 5902
- **Display:** :2
- **Password:** easybot

## 🔌 COME CONNETTERSI

1. Apri **TightVNC Viewer**
2. Inserisci: `192.168.10.191:5902`
3. Password: `easybot`

## ⚠️ NOTA

Se vedi ancora solo un terminale invece del desktop XFCE completo:

1. **Disconnetti** da VNC
2. **Riconnetti** (a volte serve un refresh)
3. Se ancora non funziona, sulla macchina esegui:
   ```bash
   vncserver -kill :2
   cp ~/.vnc/xstartup.direct ~/.vnc/xstartup
   vncserver :2 -geometry 1280x720 -depth 24 -localhost no
   ```

## 🔧 VERIFICA XFCE

Sulla macchina, verifica se XFCE è in esecuzione:
```bash
ps aux | grep xfce | grep -v grep
```

Se non ci sono processi XFCE, potrebbe essere necessario installare componenti mancanti:
```bash
sudo apt install -y xfce4 xfce4-goodies dbus-x11
```

## 💡 ALTERNATIVA: Desktop minimale funzionante

Se XFCE non funziona, puoi usare xterm (già funzionante):
```bash
vncserver -kill :2
vncserver :2 -geometry 1280x720 -depth 24 -localhost no -xstartup /usr/bin/xterm
```











