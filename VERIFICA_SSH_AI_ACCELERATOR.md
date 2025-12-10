# 🔍 VERIFICA E FIX SERVIZIO SSH - AI ACCELERATOR

## 📍 METODO 1: ACCESSO FISICO (Monitor + Tastiera)

Se hai accesso diretto alla macchina (monitor + tastiera):

### 1. Verifica stato servizio SSH
```bash
# Controlla se SSH è installato e attivo
sudo systemctl status ssh

# Output atteso se FUNZIONA:
#   ● ssh.service - OpenBSD Secure Shell server
#      Loaded: loaded (...)
#      Active: active (running) since ...
```

### 2. Se SSH NON è attivo, avvialo
```bash
# Avvia SSH
sudo systemctl start ssh

# Abilita avvio automatico al boot
sudo systemctl enable ssh

# Verifica di nuovo
sudo systemctl status ssh
```

### 3. Verifica porta SSH
```bash
# Controlla se la porta 22 è in ascolto
sudo netstat -tlnp | grep :22
# o
sudo ss -tlnp | grep :22

# Output atteso:
#   tcp  0  0  0.0.0.0:22  0.0.0.0:*  LISTEN  <pid>/sshd
```

### 4. Verifica configurazione SSH
```bash
# Controlla file di configurazione
sudo cat /etc/ssh/sshd_config | grep -E "^Port|^PermitRootLogin|^PasswordAuthentication"

# Verifica che non ci siano errori
sudo sshd -t
```

### 5. Verifica firewall
```bash
# Controlla se firewall blocca SSH
sudo ufw status

# Se firewall è attivo, permetti SSH
sudo ufw allow 22/tcp
sudo ufw reload
```

### 6. Verifica IP e rete
```bash
# Controlla IP assegnato
hostname -I
ip addr show

# Verifica che sia 192.168.10.191
# Se diverso, potrebbe essere il problema!
```

### 7. Test SSH locale
```bash
# Prova a connetterti localmente
ssh lab@localhost
# o
ssh lab@192.168.10.191
```

---

## 📍 METODO 2: VERIFICA DA REMOTO (Se hai altro accesso)

Se hai accesso via VNC o altro metodo:

### Via VNC (se funziona)
```bash
# Apri terminale nel desktop VNC e esegui:
sudo systemctl status ssh
sudo systemctl start ssh
sudo systemctl enable ssh
```

### Via serial console (se disponibile)
Collega cavo seriale e accedi alla console, poi esegui i comandi sopra.

---

## 📍 METODO 3: VERIFICA LOG E ERRORI

### Controlla log SSH
```bash
# Log SSH
sudo journalctl -u ssh -n 50
# o
sudo tail -50 /var/log/auth.log

# Cerca errori o messaggi importanti
```

### Controlla log di sistema
```bash
# Errori generali
sudo journalctl -p err -n 50

# Errori di rete
sudo journalctl -u NetworkManager -n 50
```

---

## 🔧 FIX COMPLETO SSH

Se SSH non funziona, esegui questo script completo:

```bash
#!/bin/bash
echo "=========================================="
echo "FIX COMPLETO SERVIZIO SSH"
echo "=========================================="

# 1. Installa SSH se mancante
echo "1. Verifica installazione SSH..."
if ! command -v sshd &> /dev/null; then
    echo "   ⚠️ SSH non installato, installo..."
    sudo apt update
    sudo apt install -y openssh-server
else
    echo "   ✅ SSH installato"
fi

# 2. Avvia e abilita SSH
echo "2. Avvio servizio SSH..."
sudo systemctl start ssh
sudo systemctl enable ssh

# 3. Verifica stato
echo "3. Verifica stato..."
sudo systemctl status ssh --no-pager

# 4. Verifica porta
echo "4. Verifica porta 22..."
if sudo netstat -tlnp | grep -q ":22"; then
    echo "   ✅ Porta 22 in ascolto"
else
    echo "   ❌ Porta 22 NON in ascolto"
fi

# 5. Configura firewall
echo "5. Configurazione firewall..."
sudo ufw allow 22/tcp
sudo ufw reload

# 6. Verifica IP
echo "6. Verifica IP..."
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo "   IP attuale: $CURRENT_IP"
if [ "$CURRENT_IP" = "192.168.10.191" ]; then
    echo "   ✅ IP corretto"
else
    echo "   ⚠️ IP diverso da 192.168.10.191"
fi

# 7. Test connessione locale
echo "7. Test connessione locale..."
if ssh -o ConnectTimeout=2 -o StrictHostKeyChecking=no lab@localhost exit 2>/dev/null; then
    echo "   ✅ SSH funziona localmente"
else
    echo "   ❌ SSH non funziona localmente"
fi

echo "=========================================="
echo "✅ FIX COMPLETATO"
echo "=========================================="
```

Salva come `fix_ssh_completo.sh`, rendi eseguibile e esegui:
```bash
chmod +x fix_ssh_completo.sh
bash fix_ssh_completo.sh
```

---

## 🚨 PROBLEMI COMUNI E SOLUZIONI

### Problema 1: "ssh: connect to host 192.168.10.191 port 22: Connection refused"
**Causa:** SSH non è in esecuzione
**Soluzione:**
```bash
sudo systemctl start ssh
sudo systemctl enable ssh
```

### Problema 2: "ssh: connect to host 192.168.10.191 port 22: Connection timed out"
**Causa:** Firewall blocca o IP sbagliato
**Soluzione:**
```bash
sudo ufw allow 22/tcp
sudo ufw reload
# Verifica IP: hostname -I
```

### Problema 3: "Permission denied"
**Causa:** Password o utente sbagliato
**Soluzione:**
```bash
# Verifica utente esiste
id lab
# Reimposta password se necessario
sudo passwd lab
```

### Problema 4: SSH si avvia ma si ferma subito
**Causa:** Errore di configurazione
**Soluzione:**
```bash
# Verifica configurazione
sudo sshd -t
# Controlla log
sudo journalctl -u ssh -n 50
```

---

## ✅ CHECKLIST VERIFICA SSH

- [ ] SSH installato: `which sshd`
- [ ] Servizio attivo: `sudo systemctl status ssh`
- [ ] Porta 22 in ascolto: `sudo netstat -tlnp | grep :22`
- [ ] Firewall permette SSH: `sudo ufw status`
- [ ] IP corretto: `hostname -I` (dovrebbe essere 192.168.10.191)
- [ ] Test locale: `ssh lab@localhost`
- [ ] Test remoto: `ssh lab@192.168.10.191` (da altro PC)

---

## 📋 COMANDI RAPIDI

```bash
# Stato SSH
sudo systemctl status ssh

# Avvia SSH
sudo systemctl start ssh

# Abilita SSH al boot
sudo systemctl enable ssh

# Verifica porta
sudo ss -tlnp | grep :22

# Permetti SSH nel firewall
sudo ufw allow 22/tcp

# Verifica IP
hostname -I

# Test locale
ssh lab@localhost
```





