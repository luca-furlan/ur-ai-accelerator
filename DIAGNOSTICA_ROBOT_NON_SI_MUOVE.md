# 🔍 DIAGNOSTICA: Robot Non Si Muove Nonostante Messaggi ROS2

## ✅ COSA FUNZIONA

- ✅ ROS2 bridge pubblica messaggi (`📤 Published 3750 messages`)
- ✅ Topic `/forward_velocity_controller/commands` esiste
- ✅ Publisher count: 1 (web interface pubblica)
- ✅ Subscription count: 1 (qualcuno sottoscrive)
- ✅ Formato messaggio corretto: `std_msgs/msg/Float64MultiArray`

## ❓ PROBLEMA

Il robot NON si muove nonostante i messaggi vengano pubblicati.

---

## 🔬 DIAGNOSTICA STEP-BY-STEP

### STEP 1: Verifica Controller Attivo

```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Verifica nodi controller
ros2 node list | grep controller

# Verifica topic info
ros2 topic info /forward_velocity_controller/commands
```

**Dovresti vedere:**
- Publisher count: 1 (web interface)
- Subscription count: 1 (controller)

---

### STEP 2: Test Diretto Controller

Esegui lo script di test:

```bash
cd ~/MekoAiAccelerator
./test_controller_diretto.sh
```

**Cosa fa:**
- Pubblica comando diretto: `[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]` per 2 secondi
- Dovrebbe muovere joint 0 leggermente

**Se il robot SI MUOVE:**
- ✅ Controller funziona
- ❌ Problema nella web interface o nel bridge ROS2

**Se il robot NON SI MUOVE:**
- ❌ Controller non attivo o non configurato correttamente
- Verifica prossimi step

---

### STEP 3: Verifica Robot Connesso

```bash
# Verifica stato robot
python3 verifica_stato_dopo_ethernet_ip.py
```

**Deve essere:**
- ✅ Robot in RUNNING
- ✅ Programma in PLAYING
- ✅ Remote Control abilitato
- ✅ Robot connesso al driver ROS2

---

### STEP 4: Verifica Driver ROS2

```bash
# Verifica processo driver
ps aux | grep ur_ros2_control_node

# Verifica log driver
tail -50 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -E 'controller|active|started'
```

**Cerca messaggi come:**
- `Controllers started`
- `forward_velocity_controller [active]`
- `Robot connected`

---

### STEP 5: Verifica Messaggi in Tempo Reale

In un terminale:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /forward_velocity_controller/commands
```

Poi muovi il joystick nella web interface.

**Dovresti vedere:**
- Messaggi con valori diversi da zero quando muovi il joystick
- Se vedi solo zeri → problema nel bridge ROS2

---

## 🎯 POSSIBILI CAUSE

### 1. Controller Non Attivo

**Sintomo:** Subscription count = 0 o controller non presente

**Soluzione:**
- Verifica che il driver ROS2 sia avviato correttamente
- Verifica che il robot sia connesso
- Riavvia il driver ROS2

---

### 2. Formato Messaggio Errato

**Sintomo:** Messaggi pubblicati ma controller non li processa

**Verifica:**
```bash
ros2 topic echo /forward_velocity_controller/commands --once
```

**Dovrebbe essere:**
```yaml
layout:
  dim: []
  data_offset: 0
data:
- 0.05
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```

---

### 3. Robot Non Pronto

**Sintomo:** Controller attivo ma robot non risponde

**Verifica:**
- Robot in RUNNING?
- Programma in PLAYING?
- Remote Control abilitato?
- Robot connesso al driver ROS2?

---

### 4. Velocità Troppo Basse

**Sintomo:** Messaggi con valori molto piccoli (es. 0.001)

**Verifica:**
- Le velocità nel log: `current speeds: ['-0.006', '-0.033', ...]`
- Queste sono molto piccole! Potrebbero essere sotto la soglia minima del robot

**Soluzione:**
- Aumenta le velocità nel codice joystick
- O verifica se c'è una deadzone troppo alta

---

## 🔧 SOLUZIONI

### Soluzione 1: Aumenta Velocità Joystick

Se le velocità sono troppo basse, modifica `web_interface.py`:

```javascript
const JOY_MAX = 0.1;  // Aumenta da 0.05 a 0.1
const JOY_CART_VEL = 0.02;  // Aumenta da 0.01 a 0.02
```

---

### Soluzione 2: Verifica Deadzone

La deadzone potrebbe essere troppo alta. Verifica:

```javascript
const JOY_DEADZONE = 0.15;  // Riduci se necessario
```

---

### Soluzione 3: Test Diretto

Esegui il test diretto per verificare se il controller funziona:

```bash
./test_controller_diretto.sh
```

---

## 📋 CHECKLIST COMPLETA

- [ ] Driver ROS2 attivo (`ur_ros2_control_node` in esecuzione)
- [ ] Robot connesso (vedi log driver)
- [ ] Controller attivo (subscription count > 0)
- [ ] Topic riceve messaggi (vedi con `ros2 topic echo`)
- [ ] Messaggi con valori diversi da zero quando muovi joystick
- [ ] Velocità non troppo basse (almeno 0.01 rad/s)
- [ ] Robot in RUNNING e PLAYING
- [ ] Remote Control abilitato

---

## 🆘 PROSSIMI PASSI

1. **Esegui test diretto:**
   ```bash
   ./test_controller_diretto.sh
   ```

2. **Se il test diretto funziona:**
   - Il problema è nel bridge ROS2 o nella web interface
   - Verifica messaggi in tempo reale

3. **Se il test diretto NON funziona:**
   - Il problema è nel controller o nella configurazione
   - Verifica driver ROS2 e robot connesso

