# 🔧 FIX: "receive program failed connection refused"

## 🔴 PROBLEMA

Quando provi ad attivare External Control sul Teach Pendant:
```
receive program failed connection refused
```

Anche se prima funzionava!

---

## ✅ SOLUZIONE: VERIFICA E RIPRISTINA

### Passo 1: Verifica Stato Attuale

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x verifica_stato_driver.sh
./verifica_stato_driver.sh
```

Questo ti dirà:
- ✅ Se il driver ROS2 è attivo
- ✅ Se la porta 50002 è aperta
- ✅ Se ci sono errori nel log
- ✅ Se ci sono connessioni attive

---

### Passo 2: Se Driver ROS2 NON è Attivo

**Ferma tutto e riavvia:**

```bash
# Ferma eventuali processi zombie
pkill -9 -f ur_ros2_control_node
pkill -9 -f web_interface

# Aspetta 2 secondi
sleep 2

# Verifica che tutto sia fermo
ps aux | grep -E "ur_ros2_control|web_interface"

# Riavvia driver ROS2
cd ~/MekoAiAccelerator
./avvia_driver_ros2.sh
```

**Aspetta fino a vedere:**
```
[INFO] System successfully started!
```

---

### Passo 3: Se Driver ROS2 è Attivo ma Porta 50002 è Chiusa

**Il driver potrebbe essere crashato:**

```bash
# Controlla il log per errori
tail -50 /tmp/ros2_driver.log

# Se vedi "Segmentation fault" o errori:
pkill -9 -f ur_ros2_control_node
sleep 2
./avvia_driver_ros2.sh
```

---

### Passo 4: Verifica Configurazione External Control sul Teach Pendant

**Sul Teach Pendant:**

1. Vai su **Program**
2. Apri programma con **External Control**
3. Verifica configurazione:
   - **IP Host:** `192.168.10.191` (NON 192.168.10.194!)
   - **Porta:** `50002` (NON 30002!)
   - **Host Name:** (vuoto)
4. **SALVA** il programma
5. **Attiva Remote Control** sul Teach Pendant
6. **Premi PLAY**

---

### Passo 5: Verifica Connessione

**Dalla macchina remota:**

```bash
# Verifica che la porta 50002 sia aperta
netstat -tuln | grep 50002

# Dovresti vedere:
# tcp  0  0  0.0.0.0:50002  0.0.0.0:*  LISTEN
```

**Se NON vedi questo, il driver non è avviato correttamente!**

---

## 🐛 DEBUG DETTAGLIATO

### Verifica Log Driver ROS2

```bash
tail -100 /tmp/ros2_driver.log
```

**Cerca:**
- ✅ `System successfully started!` → OK
- ❌ `Segmentation fault` → Driver crashato
- ❌ `Pipeline producer overflowed` → Problema RTDE
- ❌ `Connection refused` → Porta non aperta

### Verifica Processo Driver

```bash
ps aux | grep ur_ros2_control_node
```

**Dovresti vedere il processo attivo.**

### Verifica ROS2 Topics

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

**Dovresti vedere topics come:**
- `/joint_states`
- `/forward_velocity_controller/commands`
- `/scaled_joint_trajectory_controller/joint_trajectory`

---

## 🔄 PROCEDURA COMPLETA DI RIPRISTINO

Se nulla funziona, segui questa procedura completa:

### 1. Ferma Tutto

```bash
pkill -9 -f ur_ros2_control_node
pkill -9 -f web_interface
sleep 3
```

### 2. Verifica che Tutto sia Fermo

```bash
ps aux | grep -E "ur_ros2|web_interface"
# Non dovresti vedere processi
```

### 3. Verifica Porta 50002 Libera

```bash
netstat -tuln | grep 50002
# Non dovresti vedere nulla
```

### 4. Riavvia Driver ROS2

```bash
cd ~/MekoAiAccelerator
./avvia_driver_ros2.sh
```

**Aspetta fino a vedere:**
```
[INFO] System successfully started!
```

### 5. Verifica Porta 50002 Aperta

**In un nuovo terminale:**

```bash
ssh lab@192.168.10.191
netstat -tuln | grep 50002
# Dovresti vedere: tcp  0  0  0.0.0.0:50002  0.0.0.0:*  LISTEN
```

### 6. Sul Teach Pendant

1. Verifica configurazione External Control:
   - IP: `192.168.10.191`
   - Porta: `50002`
2. Attiva Remote Control
3. Premi PLAY sul programma External Control

### 7. Verifica Connessione

**Nel log del driver ROS2 dovresti vedere:**
```
[INFO] Connected: Universal Robots External Control
```

---

## ⚠️ ERRORI COMUNI

### "receive program failed connection refused"

**Cause possibili:**
1. Driver ROS2 non avviato → Avvia `./avvia_driver_ros2.sh`
2. Porta 50002 chiusa → Driver crashato, riavvia
3. IP sbagliato sul Teach Pendant → Verifica `192.168.10.191`
4. Porta sbagliata sul Teach Pendant → Verifica `50002`

### "Pipeline producer overflowed"

**Soluzione:** EtherNet/IP deve essere disabilitato sul robot.

### "Segmentation fault"

**Soluzione:** Usa `forward_velocity_controller` invece di `scaled_joint_trajectory_controller`.

---

## ✅ CHECKLIST FINALE

Prima di provare External Control, verifica:

- [ ] Driver ROS2 attivo (`ps aux | grep ur_ros2_control_node`)
- [ ] Porta 50002 aperta (`netstat -tuln | grep 50002`)
- [ ] Log driver mostra "System successfully started!"
- [ ] IP sul Teach Pendant: `192.168.10.191`
- [ ] Porta sul Teach Pendant: `50002`
- [ ] Remote Control attivo sul Teach Pendant
- [ ] Programma External Control in PLAYING

Se tutto è OK ma ancora non funziona, controlla il log del driver per errori specifici!

