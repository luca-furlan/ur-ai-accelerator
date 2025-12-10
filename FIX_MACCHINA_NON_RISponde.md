# 🔧 MACCHINA AI ACCELERATOR NON RISPONDE DOPO RIAVVIO

## 📊 SITUAZIONE ATTUALE
- ✅ IP **192.168.10.191** presente nella tabella ARP (rete attiva)
- ❌ Macchina **NON risponde** al ping
- ❌ Tutte le porte **CHIUSE** (SSH, VNC, Web)
- ❌ Nessun servizio raggiungibile

## 🔍 DIAGNOSI

La macchina è **sulla rete** (veduta dall'ARP) ma **non risponde**. Questo indica:

### Possibilità 1: **Macchina ancora in avvio** ⏳ (PIÙ PROBABILE)
Il **Jetson AGX Orin** può impiegare **2-5 minuti** per completare il boot, specialmente dopo un riavvio.

**Cosa fare:**
1. ⏰ **Aspetta 3-5 minuti** dal momento del riavvio
2. 🔄 **Riprova la connessione:**
   ```powershell
   ping -n 4 192.168.10.191
   ssh lab@192.168.10.191
   ```

### Possibilità 2: **Macchina in crash/kernel panic** 💥
Il sistema potrebbe essere bloccato durante il boot.

**Cosa fare:**
1. 👀 **Verifica fisicamente:**
   - LED di alimentazione accesi?
   - Ventole in funzione?
   - Display mostra qualcosa? (se presente)
   
2. 🔌 **Se possibile, collega monitor/tastiera direttamente:**
   - Vedi messaggi di errore?
   - Il sistema è bloccato?

### Possibilità 3: **Problema di rete dopo riavvio** 🌐
La configurazione di rete potrebbe non essere stata applicata correttamente.

**Cosa fare:**
1. Se hai accesso fisico, verifica:
   ```bash
   ip addr show
   # Dovrebbe mostrare IP 192.168.10.191
   ```

---

## ✅ PROCEDURA DI RISOLUZIONE

### STEP 1: Attendi e riprova (2-5 minuti)
```powershell
# Aspetta 3-5 minuti, poi:
ping -n 4 192.168.10.191
```

### STEP 2: Se ancora non risponde - Verifica fisica
- ✅ LED accesi?
- ✅ Ventole in funzione?
- ✅ Calore dalla macchina?

### STEP 3: Se hai accesso fisico (monitor/tastiera)
```bash
# Controlla stato sistema
dmesg | tail -50

# Controlla servizi
sudo systemctl status

# Controlla rete
ip addr show
ip route show

# Avvia SSH manualmente
sudo systemctl start ssh
sudo systemctl status ssh
```

### STEP 4: Se la macchina è bloccata
1. **Hard reset:** Spegni e riaccendi fisicamente
2. **Verifica alimentazione:** Cavo di alimentazione ben collegato
3. **Verifica connessione ethernet:** Cavo collegato correttamente

---

## 🚀 SOLUZIONE RAPIDA

### Se la macchina è appena stata riavviata:
1. ⏰ **Aspetta 5 minuti**
2. 🔄 **Riprova:**
   ```powershell
   python verifica_macchina_dettagliata.py
   ```
3. Se ancora non funziona, verifica fisicamente la macchina

### Se hai accesso fisico:
```bash
# Collega monitor/tastiera e verifica:
sudo systemctl start ssh
sudo systemctl enable ssh
sudo systemctl status ssh

# Verifica IP
hostname -I
# Dovrebbe mostrare: 192.168.10.191
```

---

## 📋 CHECKLIST

- [ ] Aspettato 5 minuti dopo il riavvio
- [ ] Verificato LED e ventole (macchina accesa)
- [ ] Riprovato ping: `ping -n 4 192.168.10.191`
- [ ] Riprovato SSH: `ssh lab@192.168.10.191`
- [ ] Se possibile, verificato con monitor/tastiera
- [ ] Se bloccata, provato hard reset

---

## 💡 PREVENZIONE FUTURA

Per evitare il problema in futuro, configura IP statico:

```bash
# Sulla macchina AI Accelerator (quando sarà accessibile)
sudo nano /etc/netplan/01-netcfg.yaml

# Aggiungi configurazione IP statico:
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      addresses:
        - 192.168.10.191/24
      gateway4: 192.168.10.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# Applica:
sudo netplan apply
```

---

## 🆘 SE NULLA FUNZIONA

1. **Hard reset completo:** Spegni, aspetta 30 secondi, riaccendi
2. **Verifica hardware:** LED, ventole, alimentazione
3. **Accesso diretto:** Monitor + tastiera per vedere errori
4. **Recovery mode:** Se disponibile, prova modalità recovery





