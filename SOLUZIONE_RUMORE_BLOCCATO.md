# 🔧 SOLUZIONE: Robot Fa Rumore Ma Non Si Muove

## 🔴 PROBLEMA
Il robot fa rumore "trrrrrr" ma non si muove. Questo significa:
- ✅ I comandi arrivano al robot
- ✅ I motori ricevono corrente
- ❌ Qualcosa blocca il movimento

## 🎯 CAUSE POSSIBILI

### 1. Speed Scaling Factor Troppo Basso o Zero
Il robot UR ha uno speed scaling factor che limita la velocità. Se è 0% o molto basso, il robot non si muove.

**Soluzione:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Imposta speed scaling a 100%
ros2 service call /io_and_status_controller/set_speed_slider \
    ur_msgs/srv/SetSpeedSliderFraction \
    "{speed_slider_fraction: 1.0}"
```

**OPPURE sul Teach Pendant:**
- Vai su **Settings** → **Speed Scaling**
- Imposta a **100%** o almeno **50%**

---

### 2. Brakes Attivi
I brakes dei giunti potrebbero essere ancora attivi.

**Verifica sul Teach Pendant:**
- Controlla che non ci siano indicatori di brakes attivi
- Se necessario, rilascia manualmente i brakes

---

### 3. Modalità Sicurezza
Il robot potrebbe essere in una modalità sicurezza che limita i movimenti.

**Verifica:**
```bash
# Verifica stato robot
python3 verifica_stato_dopo_ethernet_ip.py
```

**Deve essere:**
- Robot Mode: **RUNNING**
- Safety Mode: **NORMAL** (non PROTECTIVE_STOP)
- Program State: **PLAYING**

---

### 4. Velocità Troppo Bassa
La velocità potrebbe essere troppo bassa per superare attriti/brakes.

**Prova con velocità più alta:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test con velocità più alta (0.5 rad/s)
timeout 3 ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# Ferma
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

---

## ✅ PROCEDURA COMPLETA

1. **Verifica Speed Scaling sul Teach Pendant:**
   - Deve essere almeno **50%** (meglio **100%**)

2. **Imposta Speed Scaling via ROS2:**
   ```bash
   ros2 service call /io_and_status_controller/set_speed_slider \
       ur_msgs/srv/SetSpeedSliderFraction \
       "{speed_slider_fraction: 1.0}"
   ```

3. **Verifica stato robot:**
   ```bash
   python3 verifica_stato_dopo_ethernet_ip.py
   ```

4. **Test con velocità più alta:**
   ```bash
   timeout 3 ros2 topic pub -r 20 /forward_velocity_controller/commands \
       std_msgs/msg/Float64MultiArray \
       '{data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}'
   ```

5. **Se ancora non funziona:**
   - Verifica brakes sul Teach Pendant
   - Verifica che non ci siano errori di sicurezza
   - Prova con velocità ancora più alta (1.0 rad/s)

---

## 🎯 COMANDI SEMPLICI DA PROVARE

### Comando 1: Velocità media (0.3 rad/s)
```bash
ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Comando 2: Velocità alta (0.5 rad/s)
```bash
ros2 topic pub -r 20 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Comando 3: Ferma movimento
```bash
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

---

## 📋 CHECKLIST

- [ ] Speed Scaling impostato a 100% sul Teach Pendant
- [ ] Speed Scaling impostato via ROS2 service
- [ ] Robot in RUNNING mode
- [ ] Programma in PLAYING
- [ ] Nessun brake attivo
- [ ] Nessun errore sicurezza
- [ ] Velocità comando >= 0.3 rad/s

---

## 🆘 SE ANCORA NON FUNZIONA

1. Verifica log driver ROS2:
   ```bash
   tail -50 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -i error
   ```

2. Verifica topic speed scaling:
   ```bash
   ros2 topic echo /speed_scaling_status --once
   ```

3. Prova con controller diverso (scaled_joint_trajectory_controller):
   ```bash
   # Attiva controller
   ros2 service call /controller_manager/switch_controller \
       controller_manager_msgs/srv/SwitchController \
       "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: ['forward_velocity_controller'], strictness: 1}"
   
   # Test movimento
   ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory \
       trajectory_msgs/msg/JointTrajectory \
       "{joint_names: ['shoulder_pan_joint'], points: [{positions: [0.0], time_from_start: {sec: 2}}]}"
   ```

