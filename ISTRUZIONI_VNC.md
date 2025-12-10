# ✅ VNC FUNZIONANTE - ISTRUZIONI CONNESSIONE

## 📍 INFORMAZIONI CONNESSIONE

**IMPORTANTE:** VNC è attivo sulla **porta 5902** (non 5901)!

- **IP:** 192.168.10.191
- **Porta:** 5902
- **Display:** :2
- **Password:** easybot

---

## 🔌 COME CONNETTERSI

### TightVNC Viewer
1. Apri TightVNC Viewer
2. Inserisci: `192.168.10.191:5902`
3. Password: `easybot`

### RealVNC Viewer
1. Apri RealVNC Viewer
2. Inserisci: `192.168.10.191:5902`
3. Password: `easybot`

---

## ⚠️ NOTA IMPORTANTE

**Porta 5902 invece di 5901!**

Il display :1 è occupato da Xorg (GDM), quindi VNC usa il display :2 sulla porta 5902.

---

## 🔧 CONFIGURAZIONE FIREWALL

Se la connessione non funziona, configura il firewall sulla macchina:

```bash
sudo iptables -I INPUT -p tcp --dport 5902 -j ACCEPT
```

---

## 📋 VERIFICA STATO VNC

Sulla macchina, verifica:

```bash
# Lista sessioni VNC
vncserver -list

# Verifica porta
ss -tlnp | grep 5902

# Processi VNC
ps aux | grep Xtigervnc | grep -v grep
```

---

## 🚀 RIAVVIO VNC (se necessario)

```bash
# Kill e riavvia
vncserver -kill :2
vncserver :2 -geometry 1280x720 -depth 24 -localhost no -xstartup /usr/bin/xterm
```

---

## 💡 DESKTOP COMPLETO (opzionale)

Attualmente VNC usa xterm (desktop minimale). Per avere XFCE completo:

1. Fixa xstartup (vedi `fix_xstartup_vnc.sh`)
2. Installa dipendenze mancanti se necessario
3. Riavvia VNC
