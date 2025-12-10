# 🚀 COME AVVIARE IL SISTEMA COMPLETO

## 📋 ORDINE DI AVVIO

### Passo 1: Avvia Driver ROS2 (se non già attivo)

**Sulla macchina remota (192.168.10.191):**

```bash
ssh lab@192.168.10.191

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia driver ROS2
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Lascia questo terminale aperto!** Il driver deve rimanere attivo.

---

### Passo 2: Sul Teach Pendant del Robot

1. **Assicurati che il robot sia in Remote Control**
2. **Avvia il programma con External Control** (porta 50002)
3. **Lascia il programma in esecuzione**

---

### Passo 3: Avvia Web Interface

**In un NUOVO terminale sulla macchina remota:**

```bash
ssh lab@192.168.10.191

# Vai nella directory del progetto
cd ~/MekoAiAccelerator

# Avvia la web interface
./avvia_web_interface_joystick.sh
```

**OPPURE manualmente:**

```bash
cd ~/MekoAiAccelerator

# Source ROS2 (IMPORTANTE!)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Esporta variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8080

# Avvia web interface
python3 -m remote_ur_control.web_interface
```

---

## 🌐 ACCEDI ALLA WEB INTERFACE

Apri il browser su:
```
http://192.168.10.191:8080
```

---

## 🎮 COME USARE

1. **Trova lo slider** sopra i joystick: "Velocità Joystick: XX%"
2. **Regola la velocità** (10%-100%)
3. **Muovi il joystick** - il robot si muoverà alla velocità selezionata
4. **Il robot NON si muove all'avvio** - si muove solo quando muovi il joystick

---

## ✅ VERIFICA CHE TUTTO FUNZIONI

### Controlla Driver ROS2:
```bash
# Verifica che il driver sia attivo
ros2 topic list | grep joint

# Dovresti vedere:
# /joint_states
# /scaled_joint_trajectory_controller/joint_trajectory
```

### Controlla Web Interface:
```bash
# Verifica che la porta sia in ascolto
netstat -tuln | grep 8080

# Dovresti vedere:
# 0.0.0.0:8080
```

### Controlla Connessione Robot:
```bash
# Verifica porta 50002
netstat -tuln | grep 50002

# Dovresti vedere:
# 0.0.0.0:50002
```

---

## 🛑 FERMARE TUTTO

### Ferma Web Interface:
```bash
# Premi CTRL+C nel terminale della web interface
# OPPURE:
pkill -f "web_interface|flask.*8080"
```

### Ferma Driver ROS2:
```bash
# Premi CTRL+C nel terminale del driver
# OPPURE:
pkill -f "ur_robot_driver|ur_control"
```

---

## ⚠️ PROBLEMI COMUNI

### "ROS2 not available: No module named 'rclpy'"
**Soluzione**: Source ROS2 prima di avviare:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### "Porta 8080 già in uso"
**Soluzione**: Ferma il processo esistente:
```bash
pkill -f "web_interface|flask.*8080"
```

### "Robot non si muove"
**Soluzione**: 
1. Verifica che il driver ROS2 sia attivo
2. Verifica che il robot sia in Remote Control
3. Verifica che il programma External Control sia avviato sul Teach Pendant

---

## 📝 RIEPILOGO COMANDI RAPIDI

```bash
# Terminale 1: Driver ROS2
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false

# Terminale 2: Web Interface
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

**Poi apri browser**: `http://192.168.10.191:8080`

