# 🚀 COMANDI RAPIDI - Cosa Lanciare

## 📋 ORDINE DI AVVIO

### 1️⃣ AVVIA DRIVER ROS2 (Terminale 1)

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

**LASCIA QUESTO TERMINALE APERTO!** Il driver deve rimanere attivo.

---

### 2️⃣ SUL TEACH PENDANT DEL ROBOT

1. Assicurati che il robot sia in **Remote Control**
2. Avvia il programma con **External Control** (porta 50002)
3. Lascia il programma in esecuzione

---

### 3️⃣ AVVIA WEB INTERFACE (Terminale 2)

**In un NUOVO terminale:**

```bash
ssh lab@192.168.10.191

# Vai nella directory
cd ~/MekoAiAccelerator

# Avvia web interface
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

1. Trova lo slider sopra i joystick: "Velocità Joystick: XX%"
2. Regola la velocità (10%-200%)
3. Muovi il joystick - il robot si muoverà

---

## ✅ VERIFICA CHE FUNZIONI

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

## 📝 RIEPILOGO COMANDI

**Terminale 1 (Driver ROS2):**
```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false
```

**Terminale 2 (Web Interface):**
```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

**Poi apri browser:** `http://192.168.10.191:8080`

