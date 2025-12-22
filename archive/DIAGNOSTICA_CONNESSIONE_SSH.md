# 🔍 DIAGNOSTICA CONNESSIONE SSH DOPO RIAVVIO

## ❌ PROBLEMA RILEVATO
La macchina **192.168.10.191** non risponde:
- ❌ Ping fallito (100% pacchetti persi)
- ❌ Porta SSH (22) non raggiungibile

## 🔧 POSSIBILI CAUSE

### 1. **Macchina ancora in avvio** ⏳
Dopo un riavvio, la macchina può impiegare 1-2 minuti per essere completamente operativa.

**Soluzione:** Aspetta 2-3 minuti e riprova.

---

### 2. **IP cambiato dopo riavvio** 🔄
Se la macchina usa DHCP, l'IP potrebbe essere cambiato.

**Verifica:**
- Controlla il display della macchina (se presente)
- Controlla il router per vedere i dispositivi connessi
- Cerca dispositivi con hostname "ubuntu" o "jetson"

**Soluzione:** Se l'IP è cambiato, aggiorna `network_info.txt`

---

### 3. **Servizio SSH non avviato automaticamente** 🚫
Su alcune configurazioni, SSH potrebbe non avviarsi automaticamente.

**Se hai accesso fisico alla macchina:**
```bash
# Controlla se SSH è attivo
sudo systemctl status ssh

# Se non è attivo, avvialo
sudo systemctl start ssh
sudo systemctl enable ssh  # Per avviarlo automaticamente al boot
```

---

### 4. **Problema di rete/firewall** 🔥
Il firewall potrebbe bloccare le connessioni.

**Se hai accesso fisico:**
```bash
# Controlla firewall
sudo ufw status

# Se è attivo, permetti SSH
sudo ufw allow 22/tcp
```

---

### 5. **Macchina spenta o in standby** 💤
La macchina potrebbe non essere completamente accesa.

**Verifica:**
- LED di alimentazione
- Ventole in funzione
- Display (se presente)

---

## ✅ PROCEDURA DI DIAGNOSTICA

### STEP 1: Verifica connettività di base
```powershell
# Da Windows PowerShell
ping -n 4 192.168.10.191
Test-NetConnection -ComputerName 192.168.10.191 -Port 22
```

### STEP 2: Cerca altri dispositivi sulla rete
```powershell
# Scansiona la rete locale
arp -a | findstr "192.168.10"
```

### STEP 3: Se hai accesso fisico alla macchina
```bash
# Controlla IP attuale
ip addr show
# o
hostname -I

# Controlla servizio SSH
sudo systemctl status ssh

# Controlla connessioni di rete
ip route show
```

### STEP 4: Verifica configurazione di rete
```bash
# Controlla se la macchina è connessa
ip link show

# Controlla DNS
cat /etc/resolv.conf
```

---

## 🚀 SOLUZIONI RAPIDE

### Se la macchina è ancora in avvio:
1. **Aspetta 2-3 minuti**
2. **Riprova la connessione**

### Se l'IP è cambiato:
1. **Trova il nuovo IP** (router, display, scanner di rete)
2. **Aggiorna `network_info.txt`**
3. **Riprova la connessione**

### Se hai accesso fisico:
1. **Collegati direttamente** (monitor + tastiera)
2. **Verifica IP:** `hostname -I`
3. **Avvia SSH:** `sudo systemctl start ssh`
4. **Verifica:** `sudo systemctl status ssh`

---

## 📋 CHECKLIST RAPIDA

- [ ] Macchina completamente accesa (LED, ventole)
- [ ] Aspettato 2-3 minuti dopo il riavvio
- [ ] Verificato IP corretto (192.168.10.191)
- [ ] Controllato servizio SSH sulla macchina
- [ ] Verificato firewall non blocca porta 22
- [ ] Testato ping da Windows
- [ ] Testato connessione SSH

---

## 💡 PROSSIMI PASSI

1. **Se hai accesso fisico:** Collega monitor/tastiera e verifica lo stato
2. **Se non hai accesso fisico:** 
   - Verifica il router per trovare il nuovo IP
   - Usa uno scanner di rete (es. Advanced IP Scanner)
   - Controlla se altri dispositivi sulla stessa rete funzionano

3. **Una volta riconnesso:** Configura IP statico per evitare il problema in futuro:
   ```bash
   sudo nano /etc/netplan/01-netcfg.yaml
   # Configura IP statico 192.168.10.191
   sudo netplan apply
   ```











