# 🔍 VERIFICA FISICA MACCHINA AI ACCELERATOR

## 📊 STATO ATTUALE
- ✅ IP **192.168.10.191** presente in ARP (rete attiva)
- ✅ DNS risolve: `ubuntu.local` → `192.168.10.191`
- ❌ Macchina **NON risponde** a ping/SSH

## 👀 VERIFICA FISICA IMMEDIATA

### 1. LED e Indicatori
- [ ] LED di alimentazione **ACCESI**?
- [ ] LED di stato (se presenti) **lampeggianti/accesi**?
- [ ] LED ethernet **attivi**?

### 2. Ventole
- [ ] Ventole **in funzione**?
- [ ] Macchina **calda** (segno di funzionamento)?

### 3. Display (se disponibile)
- [ ] Display mostra **qualcosa**?
- [ ] Vedi messaggi di boot?
- [ ] Vedi errori o kernel panic?

---

## 🔌 SE HAI ACCESSO DIRETTO (Monitor + Tastiera)

Collega monitor e tastiera, poi:

```bash
# 1. Verifica se il sistema è avviato
dmesg | tail -20

# 2. Controlla servizi
sudo systemctl status

# 3. Avvia SSH manualmente
sudo systemctl start ssh
sudo systemctl status ssh

# 4. Verifica rete
ip addr show
hostname -I

# 5. Verifica se ci sono errori
journalctl -p err -n 50
```

---

## ⚡ AZIONI IMMEDIATE

### Se la macchina è ACCESA (LED, ventole):
1. ⏰ **Aspetta ancora 2-3 minuti** (boot in corso)
2. 🔄 **Riprova:**
   ```powershell
   ping -n 4 192.168.10.191
   ssh lab@192.168.10.191
   ```

### Se la macchina è SPENTA o BLOCCATA:
1. 🔌 **Hard reset:** Spegni completamente, aspetta 30 secondi, riaccendi
2. 👀 **Verifica alimentazione:** Cavo ben collegato?
3. 🔌 **Verifica ethernet:** Cavo collegato correttamente?

---

## 🆘 SE NULLA FUNZIONA

1. **Hard reset completo**
2. **Verifica hardware** (alimentazione, cavi)
3. **Accesso diretto** per vedere errori
4. **Recovery mode** se disponibile











