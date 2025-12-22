# ✅ RIEPILOGO INSTALLAZIONE UFFICIALE ROS2 DRIVER

**Data verifica**: $(date)  
**Fonte**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## 📊 STATO INSTALLAZIONE

### ✅ COMPLETATO

1. **ROS2 Humble** - Installato e configurato
2. **Driver ROS2 UR** - Installato via apt E compilato da sorgente
   - Pacchetti apt: `ros-humble-ur-client-library`, `ros-humble-ur-description`, etc.
   - Workspace sorgente: `~/ros2_ws/src/Universal_Robots_ROS2_Driver`
   - Compilato: ✅
3. **Launch file disponibile**: `ur_control.launch.py` ✅
4. **Robot connesso**: IP `192.168.10.194` ✅
5. **Robot in RUNNING mode** ✅

### ⚠️ DA FARE

1. **Avviare programma External Control sul teach pendant**
   - Programma attuale: `STOPPED remote_control.urp`
   - **AZIONE**: Premi PLAY sul teach pendant per mettere il programma in PLAYING

## 🚀 COME USARE IL DRIVER (SECONDO DOCUMENTAZIONE UFFICIALE)

### PASSO 1: Avvia programma sul Teach Pendant

1. Vai sul teach pendant del robot
2. Seleziona programma `remote_control.urp` (o il tuo programma con External Control)
3. Premi **PLAY**
4. Verifica che "Remote Control" sia attivo

### PASSO 2: Avvia Driver ROS2

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x AVVIA_DRIVER_UFFICIALE.sh
./AVVIA_DRIVER_UFFICIALE.sh
```

**OPPURE manualmente:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

### PASSO 3: Verifica Controller

```bash
# Installa se necessario
sudo apt install ros-humble-ros2controlcli

# Verifica controller
ros2 control list_controllers
```

Dovresti vedere controller attivi come:
- `scaled_joint_trajectory_controller [active]`
- `forward_velocity_controller [active]`
- `joint_state_broadcaster [active]`

### PASSO 4: Muovi il Robot

**Opzione A: Test automatico (se disponibile)**
```bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

**Opzione B: Comando manuale**
```bash
ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.0, -1.57, 1.57, -1.57, 1.57, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}'
```

**Opzione C: Web Interface**
```bash
python3 remote_ur_control/web_interface.py
```

## 📋 COMANDI UTILI

### Verifica installazione
```bash
./VERIFICA_INSTALLAZIONE_UFFICIALE.sh
```

### Test controllo
```bash
./TEST_CONTROLLO_UFFICIALE.sh
```

### Verifica topic ROS2
```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /scaled_joint_trajectory_controller/joint_trajectory
```

## 🔗 RIFERIMENTI UFFICIALI

- **Repository GitHub**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **Getting Started**: Vedi README.md del repository
- **Usage Guide**: Vedi `ur_robot_driver/doc/usage.rst` nel repository

## ⚠️ NOTE IMPORTANTI

1. **Programma in PLAYING obbligatorio** - Il programma External Control DEVE essere in PLAYING sul teach pendant prima di avviare il driver
2. **Calibrazione** - Per TCP pose corretto, estrai calibrazione con:
   ```bash
   ros2 launch ur_calibration calibration_correction.launch.py \
       robot_ip:=192.168.10.194 \
       target_filename:${HOME}/my_robot_calibration.yaml
   ```
3. **Linux obbligatorio** - Il driver funziona solo su Linux
4. **External Control URCap** - Deve essere installato e configurato sul robot









