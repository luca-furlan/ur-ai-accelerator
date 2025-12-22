# 📚 GUIDA UFFICIALE - Universal Robots ROS2 Driver

**Fonte**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## 🎯 Setup Completo Secondo Documentazione Ufficiale

### PASSO 1: Installazione Driver

**Opzione A: Via apt (più semplice)**
```bash
sudo apt-get install ros-humble-ur
```

**Opzione B: Da sorgente (se apt non disponibile)**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### PASSO 2: Setup Robot (Teach Pendant)

**IMPORTANTE**: Segui questi passi sul teach pendant del robot:

1. **Installa External Control URCap**
   - Vai su: **Installation** → **URCaps**
   - Installa **"External Control"**

2. **Crea Programma con External Control**
   - Crea nuovo programma
   - Aggiungi nodo **"External Control"**
   - Configura:
     - **IP Host**: `192.168.10.191` (IP AI Accelerator)
     - **Porta**: `50002` (default)
   - Salva programma (es. `ros_control.urp`)

3. **Estrai Calibrazione** (IMPORTANTE per TCP pose corretto)
   ```bash
   ros2 launch ur_calibration calibration_correction.launch.py \
       robot_ip:=192.168.10.194 \
       target_filename:${HOME}/my_robot_calibration.yaml
   ```

4. **Avvia Programma**
   - Metti programma in **PLAYING**
   - Verifica che "Remote Control" sia attivo

### PASSO 3: Avvia Driver

**Comando ufficiale dalla documentazione GitHub:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # se compilato da sorgente

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Aspetta che vedi:**
```
[INFO] [ur_robot_driver]: Robot connected
[INFO] [ur_robot_driver]: Controllers started
```

### PASSO 4: Verifica Controller

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
sudo apt install ros-humble-ros2controlcli
ros2 control list_controllers
```

**Dovresti vedere controller attivi come:**
- `scaled_joint_trajectory_controller [active]`
- `forward_velocity_controller [active]`
- `joint_state_broadcaster [active]`
- etc.

### PASSO 5: Muovi il Robot

**Secondo documentazione ufficiale:**

1. **Verifica controller attivi:**
   ```bash
   ros2 control list_controllers
   ```

2. **Test movimento con demo node:**
   ```bash
   ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
   ```
   
   Il robot dovrebbe muoversi dopo pochi secondi.

3. **Oppure usa esempio Python:**
   ```bash
   ros2 run ur_robot_driver example_move.py
   ```

## 📋 Controller Disponibili (dalla documentazione)

Secondo il file `ur_controllers.yaml` trovato:

- `scaled_joint_trajectory_controller` - Per movimenti con traiettorie
- `forward_velocity_controller` - Per controllo velocità diretto
- `joint_trajectory_controller` - Per traiettorie standard
- `forward_position_controller` - Per controllo posizione
- `forward_effort_controller` - Per controllo coppia

## 🎮 Topic ROS2 Disponibili

Quando il driver è in esecuzione, dovresti vedere:

- `/scaled_joint_trajectory_controller/joint_trajectory` - Per traiettorie
- `/forward_velocity_controller/commands` - Per controllo velocità
- `/joint_states` - Stato joint del robot
- `/tf` - Trasformazioni
- `/io_and_status_controller/...` - IO e stato

## ⚠️ Note Importanti dalla Documentazione

1. **Linux obbligatorio** - Il driver funziona solo su Linux
2. **Calibrazione importante** - Estrai calibrazione per TCP pose corretto
3. **External Control URCap obbligatorio** - Serve per ROS2 control
4. **Programma in PLAYING** - Il programma External Control deve essere in PLAYING

## 🔗 Riferimenti

- **Repository GitHub**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **Documentazione**: Vedi cartella `doc/` nel repository
- **Installation Guide**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#getting-started
- **Usage Guide**: Vedi `ur_robot_driver/doc/usage.rst` nel repository










