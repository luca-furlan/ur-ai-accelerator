# 🔧 SOLUZIONE: Brakes Bloccati - Robot Fa Rumore Ma Non Si Muove

## 🔴 PROBLEMA
Robot fa rumore "grrr rnrrrrr" ma non si muove fisicamente, anche con speed scaling al 100%.

## 🎯 CAUSA PRINCIPALE: Brakes dei Giunti Non Rilasciati

I robot UR hanno **brakes elettromagnetici** su ogni giunto che:
- Si **attivano** quando il robot è spento
- Si **rilasciano** quando il robot è acceso e in RUNNING mode
- Se non si rilasciano correttamente, i giunti rimangono bloccati

---

## ✅ SOLUZIONI

### Soluzione 1: Riavvia Robot (Rilascia Brakes)

1. **Sul Teach Pendant:**
   - Vai su **Shutdown** o **Power Off**
   - Attendi **10 secondi**
   - **Riavvia** il robot
   - **Ascolta il "click"** quando i brakes si rilasciano

2. **Verifica:**
   - Dovresti sentire un rumore meccanico quando i brakes si rilasciano
   - Se non senti nulla, i brakes potrebbero essere ancora attivi

---

### Soluzione 2: Verifica Modalità Robot

**Sul Teach Pendant verifica:**
- **Robot Mode**: Deve essere **RUNNING** (non POWER_OFF o INIT)
- **Safety Mode**: Deve essere **NORMAL** (non PROTECTIVE_STOP)
- **Program State**: Deve essere **PLAYING**

Se è in PROTECTIVE_STOP:
- Risolvi l'errore di sicurezza
- Reset sicurezza
- Riavvia programma

---

### Soluzione 3: Test Movimento Manuale

**Sul Teach Pendant:**
1. Prova a muovere il robot **manualmente** usando i tasti
2. Se **NON riesci** a muoverlo manualmente:
   - Problema **hardware** (brakes, motori, meccanica)
   - Controlla brakes fisicamente
3. Se **riesci** manualmente ma non via ROS2:
   - Problema **software** (controller, configurazione ROS2)

---

### Soluzione 4: Verifica Ostacoli Fisici

**Controlla:**
- Nessun oggetto che blocca i giunti
- Nessun cavo che impedisce movimento
- Giunti si muovono liberamente quando spinti (con brakes rilasciati)

---

### Soluzione 5: Test con Velocità Molto Alta

Prova con velocità molto alta per vedere se riesce a superare i brakes:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test con velocità molto alta (2.0 rad/s)
timeout 2 ros2 topic pub -r 50 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# Ferma
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

**⚠️ ATTENZIONE**: Usa velocità alta solo per test, non per uso normale!

---

## 📋 CHECKLIST COMPLETA

### Sul Teach Pendant:
- [ ] Robot in **RUNNING** mode
- [ ] Safety Mode = **NORMAL**
- [ ] Programma in **PLAYING**
- [ ] Speed Scaling = **100%**
- [ ] Nessun errore o warning
- [ ] Brakes rilasciati (sentito 'click' all'accensione)

### Verifica Fisica:
- [ ] Nessun ostacolo fisico
- [ ] Giunti si muovono liberamente (con brakes rilasciati)
- [ ] Nessun cavo che impedisce movimento

### Test:
- [ ] Movimento manuale funziona sul Teach Pendant?
- [ ] Movimento via ROS2 funziona?

---

## 🔍 DIAGNOSTICA AVANZATA

### Verifica Log Driver ROS2:
```bash
tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -i error
```

### Verifica Stato Robot Completo:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 verifica_stato_dopo_ethernet_ip.py
```

### Test Controller Diverso:
Se `forward_velocity_controller` non funziona, prova con `scaled_joint_trajectory_controller`:

```bash
# Attiva controller
ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: ['forward_velocity_controller'], strictness: 1}"

# Test movimento posizione
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: ['shoulder_pan_joint'], points: [{positions: [0.0], time_from_start: {sec: 2}}]}"
```

---

## 🆘 SE ANCORA NON FUNZIONA

1. **Contatta supporto Universal Robots** con:
   - Modello robot (UR5e)
   - Versione firmware
   - Log errori
   - Descrizione problema

2. **Verifica hardware:**
   - Controlla connessioni elettriche
   - Verifica che i brakes funzionino correttamente
   - Controlla che i motori ricevano corrente

3. **Problema noto:**
   - Alcuni robot UR hanno problemi con brakes che non si rilasciano
   - Potrebbe essere necessario intervento tecnico

---

## 💡 NOTA IMPORTANTE

Il rumore "grrr rnrrrrr" indica che:
- ✅ Comandi ROS2 arrivano correttamente
- ✅ Controller è attivo
- ✅ Motori ricevono corrente
- ❌ Qualcosa blocca fisicamente il movimento

**La causa più probabile sono i brakes non rilasciati correttamente.**

