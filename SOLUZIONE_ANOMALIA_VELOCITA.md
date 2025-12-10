# 🔧 SOLUZIONE: Anomalia Velocità Giunti - Robot UR ROS2

## 🔴 PROBLEMA
Robot va in anomalia quando prova a muoversi con errori di velocità sui giunti.

## 🎯 CAUSE TROVATE ONLINE

### 1. Conflitti con Altri Client RTDE ⚠️ **PROBABILE CAUSA**
Altri client RTDE potrebbero controllare variabili come `speed_slider_mask`, causando conflitti.

**Soluzione:**
- Disabilita **Fieldbus** (EtherNet/IP, Modbus, etc.) se attivo
- Rimuovi **URCap non necessari** che potrebbero interferire
- Verifica che solo il driver ROS2 controlli il robot

---

### 2. Velocità Troppo Alta
Velocità troppo alta può causare anomalie di sicurezza.

**Soluzione:**
- Usa velocità **MOLTO BASSA** per iniziare (0.05-0.1 rad/s)
- Aumenta gradualmente se funziona
- Non superare **0.5 rad/s** inizialmente

---

### 3. Limiti di Sicurezza Attivi
Limiti di velocità o posizione configurati nel sistema di sicurezza.

**Verifica sul Teach Pendant:**
- Settings → Safety → Joint Limits
- Settings → Safety → Speed Limits
- Assicurati che non ci siano limiti troppo restrittivi

---

### 4. Configurazione Controller Errata
Il controller potrebbe avere limiti di velocità configurati male.

**Verifica:**
- File di configurazione controller (`ur_controllers.yaml`)
- Limiti di velocità massima per joint

---

## ✅ SOLUZIONI IMMEDIATE

### Soluzione 1: Riduci Velocità Massima

**Modifica web interface per usare velocità più basse:**

Nel file `web_interface.py`, cerca:
```javascript
const JOY_MAX = 0.15;  // max joint velocity (rad/s)
```

**Cambia in:**
```javascript
const JOY_MAX = 0.05;  // max joint velocity (rad/s) - RIDOTTO per evitare anomalie
```

---

### Soluzione 2: Disabilita Fieldbus

**Sul Teach Pendant:**
1. Vai su **Installation** → **Fieldbus**
2. **Disabilita** EtherNet/IP se attivo
3. **Disabilita** altri fieldbus non necessari
4. Riavvia robot

**Questo è spesso la causa principale del problema!**

---

### Soluzione 3: Test con Velocità Molto Bassa

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test con velocità MOLTO BASSA (0.05 rad/s)
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# Ferma
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

---

### Soluzione 4: Verifica Configurazione Controller

**Controlla limiti velocità nel file di configurazione:**

```bash
# Trova file configurazione controller
find ~/ros2_ws -name "ur_controllers.yaml" -o -name "*controller*.yaml" | grep ur

# Verifica limiti velocità configurati
```

---

## 📋 PROCEDURA COMPLETA

### Passo 1: Risolvi Anomalia
- Sul Teach Pendant: **Reset anomalia**
- Riavvia programma

### Passo 2: Disabilita Fieldbus
- **Installation** → **Fieldbus**
- Disabilita **EtherNet/IP** e altri fieldbus
- Riavvia robot

### Passo 3: Riduci Velocità Web Interface
- Modifica `JOY_MAX` da `0.15` a `0.05` rad/s
- Riavvia web interface

### Passo 4: Test con Velocità Bassa
```bash
cd ~/MekoAiAccelerator
./fix_anomalia_velocita.sh
```

### Passo 5: Se Funziona, Aumenta Gradualmente
- Se 0.05 rad/s funziona, prova 0.1 rad/s
- Poi 0.15 rad/s
- Non superare 0.5 rad/s inizialmente

---

## 🎯 COMANDI TEST VELOCITÀ BASSA

### Test 1: Velocità molto bassa (0.05 rad/s)
```bash
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Test 2: Velocità bassa (0.1 rad/s)
```bash
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Test 3: Velocità media (0.15 rad/s)
```bash
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.15, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

---

## ⚠️ IMPORTANTE

**La causa più probabile è il conflitto con Fieldbus (EtherNet/IP).**

**Soluzione principale:**
1. **Disabilita EtherNet/IP** sul Teach Pendant
2. **Riavvia robot**
3. **Riprova con velocità bassa**

---

## 🔍 VERIFICA LOG ERRORI

```bash
# Log driver ROS2
tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -i error

# Log robot (se disponibile)
tail -50 ~/.ros/log/latest/*.log | grep -i "speed\|velocity\|anomaly"
```

---

## 📞 SE ANCORA NON FUNZIONA

1. Contatta supporto Universal Robots con:
   - Modello robot (UR5e)
   - Versione firmware
   - Log errori
   - Descrizione problema (anomalia velocità giunti)

2. Verifica documentazione ufficiale:
   - Manuale utente UR5e
   - Documentazione ROS2 driver UR

