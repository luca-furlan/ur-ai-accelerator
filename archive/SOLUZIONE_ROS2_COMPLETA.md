# 🔧 SOLUZIONE COMPLETA ROS2 - Documentazione Ufficiale

## ❓ Perché la Porta 50002 è Chiusa?

La porta 50002 è chiusa perché:
1. **Driver UR ROS2 non è in esecuzione** sull'AI Accelerator
2. **Robot non è configurato** con External Control URCap
3. **Programma con External Control non è in PLAYING**

## ✅ Soluzione Completa ROS2

### PASSO 1: Configura Robot (Teach Pendant) - **OBBLIGATORIO**

1. **Installa External Control URCap**
   - Vai su: **Installation** → **URCaps**
   - Installa **"External Control"**

2. **Crea Programma con External Control**
   - Crea nuovo programma
   - Aggiungi nodo **"External Control"**
   - Configura:
     - **IP Host**: `192.168.10.191` (IP AI Accelerator)
     - **Porta**: `50002`
   - Salva come `ros_control.urp`

3. **Avvia Programma**
   - Metti in **PLAYING**
   - Verifica che "Remote Control" sia attivo
   - **IMPORTANTE**: La porta 50002 si aprirà solo quando il programma è in PLAYING!

### PASSO 2: Avvia Driver UR ROS2 (AI Accelerator)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x AVVIA_ROS2_DRIVER.sh
./AVVIA_ROS2_DRIVER.sh
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

**Aspetta che vedi:**
```
[INFO] [ur_robot_driver]: Robot connected
[INFO] [ur_robot_driver]: Controllers started
```

### PASSO 3: Verifica Controller ROS2

In un altro terminale:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
sudo apt install ros-humble-ros2controlcli  # se non installato
ros2 control list_controllers
```

**Dovresti vedere:**
```
scaled_joint_trajectory_controller [active]
forward_velocity_controller [active]
...
```

### PASSO 4: Test Movimento ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

Il robot dovrebbe muoversi dopo pochi secondi.

### PASSO 5: Riavvia Web Interface con ROS2

```bash
ssh lab@192.168.10.191
pkill -f web_interface
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

**La web interface ora userà ROS2 invece di socket!**

## 📋 Checklist Completa

- [ ] External Control URCap installato sul robot
- [ ] Programma con External Control creato
- [ ] IP Host configurato: 192.168.10.191
- [ ] Porta configurata: 50002
- [ ] Programma in PLAYING
- [ ] Porta 50002 aperta (verifica con: `nc -zv 192.168.10.194 50002`)
- [ ] Driver UR ROS2 in esecuzione
- [ ] Controller ROS2 attivi (`ros2 control list_controllers`)
- [ ] Web interface riavviata con ROS2 configurato

## 🐛 Debug

### Verifica Porta 50002

```bash
# Da AI Accelerator
nc -zv 192.168.10.194 50002
```

Se aperta: `Connection to 192.168.10.194 50002 port [tcp/*] succeeded!`

### Verifica Topic ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep forward_velocity
ros2 topic echo /forward_velocity_controller/commands
```

Se vedi messaggi quando muovi joystick → ROS2 funziona!

### Verifica Driver

```bash
ros2 node list | grep ur_robot_driver
```

Dovresti vedere nodi del driver UR ROS2.

## 💡 Note Importanti

1. **Il driver UR ROS2 DEVE essere in esecuzione** prima di usare ROS2
2. **Il robot DEVE essere configurato** con External Control URCap
3. **Il programma DEVE essere in PLAYING** per aprire la porta 50002
4. **La web interface usa ROS2 per default**, fallback a socket se ROS2 non disponibile

## 🔄 Setup Completo (3 Terminali)

**TERMINALE 1 - Driver UR ROS2:**
```bash
./AVVIA_ROS2_DRIVER.sh
```

**TERMINALE 2 - Web Interface:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

**TERMINALE 3 - Monitor:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /forward_velocity_controller/commands
```

## ✅ Verifica Funzionamento

1. Apri web interface: http://192.168.10.191:8081
2. Muovi joystick
3. Nel Terminale 3 dovresti vedere messaggi ROS2
4. Il robot dovrebbe muoversi!










