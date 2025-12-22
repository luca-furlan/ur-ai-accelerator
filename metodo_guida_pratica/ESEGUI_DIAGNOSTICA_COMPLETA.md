# 🔍 GUIDA: ESECUZIONE DIAGNOSTICA COMPLETA

## Script disponibili

1. **DIAGNOSTICA_COMPLETA_RETE.sh** - Diagnostica completa da PC Windows
2. **DIAGNOSTICA_ROBOT.sh** - Diagnostica specifica ROBOT
3. **DIAGNOSTICA_AI_ACCELERATOR.sh** - Diagnostica specifica AI Accelerator

---

## Come eseguire

### Opzione 1: Da PC Windows (diagnostica completa)

```bash
cd metodo_guida_pratica

# Installa sshpass se non presente
# Su Windows WSL/Linux:
sudo apt-get install sshpass

# Esegui diagnostica completa
./DIAGNOSTICA_COMPLETA_RETE.sh
```

### Opzione 2: Da AI Accelerator

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator/metodo_guida_pratica

# Diagnostica AI Accelerator
./DIAGNOSTICA_AI_ACCELERATOR.sh

# Diagnostica ROBOT (richiede sshpass)
sudo apt-get install sshpass
./DIAGNOSTICA_ROBOT.sh
```

### Opzione 3: Copia script su AI Accelerator e esegui

```bash
# Da PC Windows
scp metodo_guida_pratica/DIAGNOSTICA_AI_ACCELERATOR.sh lab@192.168.10.191:~/MekoAiAccelerator/metodo_guida_pratica/
scp metodo_guida_pratica/DIAGNOSTICA_ROBOT.sh lab@192.168.10.191:~/MekoAiAccelerator/metodo_guida_pratica/

# Poi su AI Accelerator
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator/metodo_guida_pratica
chmod +x *.sh
./DIAGNOSTICA_AI_ACCELERATOR.sh
```

---

## Cosa verificano gli script

### DIAGNOSTICA_COMPLETA_RETE.sh
- ✅ Connettività tra tutte le macchine
- ✅ SSH accessibile
- ✅ Porte aperte (50002, 30001, 30002, 29999)
- ✅ Processi ROS2 attivi
- ✅ Firewall configurato
- ✅ Configurazione rete

### DIAGNOSTICA_ROBOT.sh
- ✅ Informazioni sistema robot
- ✅ Porte aperte sul robot
- ✅ Processi URCap attivi
- ✅ Connettività verso AI Accelerator

### DIAGNOSTICA_AI_ACCELERATOR.sh
- ✅ Informazioni sistema
- ✅ Porte in ascolto
- ✅ Processi ROS2
- ✅ Firewall
- ✅ Connettività verso robot
- ✅ Servizi SSH

---

## Risoluzione problemi

### Se SSH non funziona su AI Accelerator:

```bash
# Su AI Accelerator
sudo systemctl status ssh
sudo systemctl start ssh
sudo systemctl enable ssh
```

### Se sshpass non è installato:

```bash
# Su Ubuntu/Debian
sudo apt-get install sshpass

# Su Windows WSL
sudo apt-get install sshpass
```

### Se robot non raggiungibile:

1. Verifica connessione fisica
2. Verifica IP robot: 192.168.10.194
3. Verifica subnet: tutte le macchine devono essere su 192.168.10.x

---

## Output atteso

Gli script mostrano:
- ✅ = OK
- ❌ = Problema trovato
- ⚠️ = Avviso

Alla fine c'è un riepilogo con azioni consigliate.








