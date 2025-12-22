# 🔧 FIX BLOCCHI SSH - SSH Attivo Ma Non Raggiungibile

## 📊 SITUAZIONE
- ✅ SSH service **ATTIVO** sulla macchina
- ✅ IP corretto: **192.168.10.191**
- ❌ **NON raggiungibile** da remoto

## 🔍 POSSIBILI CAUSE

### 1. **Firewall UFW blocca SSH**
```bash
# Verifica
sudo ufw status

# Se UFW è attivo ma SSH non è permesso:
sudo ufw allow 22/tcp
sudo ufw reload
```

### 2. **IPTABLES blocca SSH**
```bash
# Verifica
sudo iptables -L -n | grep 22

# Permetti SSH
sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT
sudo iptables-save | sudo tee /etc/iptables/rules.v4
```

### 3. **SSH ascolta solo su localhost**
```bash
# Verifica configurazione
sudo grep "ListenAddress" /etc/ssh/sshd_config

# Se mostra "ListenAddress 127.0.0.1", fixa:
sudo sed -i 's/^ListenAddress.*/#ListenAddress 0.0.0.0/' /etc/ssh/sshd_config
sudo systemctl restart ssh
```

### 4. **Password authentication disabilitata**
```bash
# Verifica
sudo grep "PasswordAuthentication" /etc/ssh/sshd_config

# Se è "no", abilita:
sudo sed -i 's/^PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config
sudo systemctl restart ssh
```

### 5. **Router/Switch blocca porta 22**
- Verifica configurazione router
- Controlla se c'è un firewall hardware

---

## ✅ SCRIPT AUTOMATICO

Esegui lo script `fix_blocchi_ssh.sh` che verifica e fixa tutto automaticamente:

```bash
chmod +x fix_blocchi_ssh.sh
bash fix_blocchi_ssh.sh
```

---

## 🔧 COMANDI RAPIDI (tutti insieme)

```bash
# 1. Permetti SSH in UFW
sudo ufw allow 22/tcp
sudo ufw reload

# 2. Permetti SSH in IPTABLES
sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT

# 3. Verifica SSH ascolta su tutte le interfacce
sudo sed -i 's/^ListenAddress.*/#ListenAddress 0.0.0.0/' /etc/ssh/sshd_config

# 4. Abilita password authentication
sudo sed -i 's/^PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config

# 5. Riavvia SSH
sudo systemctl restart ssh

# 6. Verifica
sudo ss -tlnp | grep :22
sudo ufw status
```

---

## 🧪 TEST

Dopo i fix, testa:

```bash
# Dalla macchina stessa
ssh lab@localhost

# Da un altro PC sulla stessa rete
ssh lab@192.168.10.191
```

---

## 📋 CHECKLIST

- [ ] UFW permette porta 22: `sudo ufw allow 22/tcp`
- [ ] IPTABLES permette porta 22: `sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT`
- [ ] SSH ascolta su 0.0.0.0 (non solo 127.0.0.1)
- [ ] Password authentication abilitata
- [ ] SSH riavviato: `sudo systemctl restart ssh`
- [ ] Porta 22 in ascolto: `sudo ss -tlnp | grep :22`
- [ ] Test locale: `ssh lab@localhost`
- [ ] Test remoto: `ssh lab@192.168.10.191` (da altro PC)











