# 🔧 FIX DETTAGLIATO: Connection Refused sulla Porta 50002

## 🔴 PROBLEMA

Quando avvii il driver ROS2 e poi attivi il programma External Control sul Teach Pendant:
```
connection refused
```

**Causa**: La porta 50002 non si apre correttamente o il driver crasha prima che la porta si apra.

---

## ✅ VERIFICA COMPLETA

### Passo 1: Verifica Porta 50002

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x verifica_porta_50002.sh
./verifica_porta_50002.sh
```

Questo script verifica:
- ✅ Se il driver ROS2 è attivo
- ✅ Se la porta 50002 è aperta (netstat)
- ✅ Se la porta 50002 è aperta (ss)
- ✅ Se la porta 50002 è raggiungibile (test TCP)
- ✅ Errori nel log del driver
- ✅ Processi che usano la porta 50002

---

## 🔍 DIAGNOSI PROBLEMI COMUNI

### Problema 1: Driver Attivo ma Porta NON Aperta

**Sintomi:**
- `ps aux | grep ur_ros2_control_node` → Processo attivo
- `netstat -tuln | grep 50002` → Nessun risultato

**Cause possibili:**
1. Driver crashato subito dopo l'avvio
2. Driver non ha completato l'inizializzazione
3. Problema con la configurazione del driver

**Soluzione:**
```bash
# Controlla log completo
tail -100 /tmp/ros2_driver.log

# Cerca errori specifici
grep -i "error\|fatal\|segmentation\|crash" /tmp/ros2_driver.log

# Se vedi errori, ferma e riavvia
pkill -9 -f ur_ros2_control_node
sleep 2
./avvia_driver_ros2.sh
```

---

### Problema 2: Driver Crasha Subito Dopo Avvio

**Sintomi:**
- Driver si avvia ma poi scompare
- Porta 50002 non si apre mai
- Log mostra "Segmentation fault" o altri errori

**Soluzione:**
```bash
# Verifica log per errori
tail -50 /tmp/ros2_driver.log

# Se vedi "Segmentation fault":
# - Usa forward_velocity_controller invece di scaled_joint_trajectory_controller
# - Verifica che EtherNet/IP sia disabilitato sul robot
# - Verifica che non ci siano altri processi RTDE attivi
```

---

### Problema 3: Porta 50002 Occupata da Altro Processo

**Sintomi:**
- `netstat -tuln | grep 50002` mostra un processo diverso
- Driver ROS2 non può aprire la porta

**Soluzione:**
```bash
# Trova processo che usa porta 50002
lsof -i :50002
# OPPURE
netstat -tulnp | grep 50002

# Kill processo (se non è il driver ROS2)
kill -9 <PID>

# Riavvia driver ROS2
./avvia_driver_ros2.sh
```

---

## 📋 PROCEDURA COMPLETA DI RISOLUZIONE

### 1. Ferma Tutto

```bash
pkill -9 -f ur_ros2_control_node
pkill -9 -f web_interface
sleep 3
```

### 2. Verifica Porta Libera

```bash
netstat -tuln | grep 50002
# Non dovresti vedere nulla
```

### 3. Avvia Driver ROS2

```bash
cd ~/MekoAiAccelerator
./avvia_driver_ros2.sh
```

**Aspetta fino a vedere:**
```
[INFO] System successfully started!
```

### 4. Verifica Porta 50002 Aperta

**In un nuovo terminale:**
```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
./verifica_porta_50002.sh
```

**Dovresti vedere:**
```
✅ Porta 50002 APERTA
✅ Porta 50002 RAGGIUNGIBILE
✅ TUTTO OK: Driver attivo e porta 50002 aperta
```

### 5. Sul Teach Pendant

**SOLO DOPO che la porta 50002 è aperta:**

1. Verifica configurazione External Control:
   - **IP Host:** `192.168.10.191`
   - **Porta:** `50002`
2. Attiva Remote Control
3. Premi PLAY sul programma External Control

---

## ⚠️ ERRORI COMUNI NEL LOG

### "Segmentation fault"
**Soluzione:** Usa `forward_velocity_controller` invece di `scaled_joint_trajectory_controller`

### "Pipeline producer overflowed"
**Soluzione:** Disabilita EtherNet/IP sul robot

### "Connection refused" nel log driver
**Soluzione:** Verifica che il robot sia in Remote Control e che il programma External Control sia configurato correttamente

### "Port already in use"
**Soluzione:** Kill processo esistente sulla porta 50002

---

## ✅ CHECKLIST FINALE

Prima di attivare External Control sul Teach Pendant:

- [ ] Driver ROS2 attivo (`ps aux | grep ur_ros2_control_node`)
- [ ] Porta 50002 aperta (`netstat -tuln | grep 50002`)
- [ ] Log mostra "System successfully started!"
- [ ] Nessun errore nel log (`tail -50 /tmp/ros2_driver.log`)
- [ ] Test connessione TCP riuscito (`timeout 2 bash -c "echo > /dev/tcp/127.0.0.1/50002"`)

Se tutto è OK ma ancora "connection refused", verifica configurazione sul Teach Pendant!

