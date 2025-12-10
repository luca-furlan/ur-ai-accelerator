# 🖥️ GUIDA COMPLETA ACCESSO E UTILIZZO AI ACCELERATOR

## 📍 INFORMAZIONI ACCESSO

### Credenziali di Rete
- **IP AI Accelerator**: `192.168.10.191`
- **IP Robot UR**: `192.168.10.194`
- **Utente SSH**: `lab`
- **Password SSH**: `easybot`
- **Porta VNC**: `5901` (se disponibile)

### Versione AI Accelerator
- **Hardware**: NVIDIA Jetson AGX Orin (64GB) - AI Accelerator 1.0/1.1
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill

---

## 🔌 MODALITÀ DI ACCESSO

### 1. SSH (Terminale)
**Comando:**
```bash
ssh lab@192.168.10.191
# Password: easybot
```

**Cosa puoi fare:**
- Controllo completo via terminale
- Eseguire comandi ROS2
- Avviare servizi
- Modificare file
- Monitorare processi

### 2. VNC (Desktop Remoto)
**Se VNC è configurato:**
```bash
# Connessione VNC
# IP: 192.168.10.191:5901
# Utente: lab
# Password: easybot
```

**Client VNC consigliati:**
- **Windows**: TightVNC Viewer, RealVNC Viewer
- **Mac**: Built-in Screen Sharing (vnc://192.168.10.191:5901)
- **Linux**: Remmina, TigerVNC

**Verifica se VNC è attivo:**
```bash
ssh lab@192.168.10.191 "systemctl status vncserver@* || ps aux | grep vnc"
```

### 3. X11 Forwarding (GUI via SSH)
**Comando:**
```bash
ssh -X lab@192.168.10.191
# Poi puoi avviare applicazioni GUI
```

---

## 📦 COSA C'È INSTALLATO

### ✅ Componenti Base
- **Ubuntu 22.04 LTS** - Sistema operativo
- **ROS 2 Humble** - Framework robotica
- **Python 3.10** - Linguaggio programmazione
- **Docker** - Containerizzazione

### ✅ Driver Universal Robots
- **Universal_Robots_ROS2_Driver** - Driver ufficiale UR
- **ur_rtde** (v1.6.2) - Interfaccia real-time
- **ros2_control** - Controllo robot

### ✅ Simulazione MuJoCo
- **MuJoCo** (v3.3.7) - Physics engine
- **MuJoCo Menagerie** - Modelli robot
  - UR5e model
  - UR10e model

### ✅ AI e Object Detection
- **YOLOv8** (v8.3.235) - Object detection
- **PyTorch** (v2.9.1) - Deep learning
- **OpenCV** (v4.12.0.88) - Computer vision
- **Open3D** (v0.18.0) - Point cloud processing

### ✅ Motion Planning
- **MoveIt2** (15 pacchetti) - Motion planning
- **ros2_control** - Controllo movimento

### ✅ Camera Orbbec
- **OrbbecSDK_ROS2** - Driver camera
- **orbbec_camera** - Nodo ROS2 camera
- **orbbec_camera_msgs** - Messaggi ROS2

### ✅ Componenti Custom
- **Web Interface** - Controllo robot via browser
- **ROS2 Bridge** - Bridge comandi
- **Remote UR Controller** - Controller remoto

---

## 🚀 COME UTILIZZARE IL SISTEMA

### 1. Setup Ambiente ROS2

**Via SSH:**
```bash
ssh lab@192.168.10.191

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Variabili ambiente
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8081
```

### 2. Avviare Driver Robot UR

```bash
# Source ambiente
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=true
```

**Cosa vedi:**
- Driver ROS2 attivo
- Topic ROS2 disponibili
- RViz (se launch_rviz:=true)

### 3. Avviare Camera Orbbec

```bash
# Source ambiente
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia camera
ros2 launch orbbec_camera gemini_330_series.launch.py
```

**Topic disponibili:**
- `/camera/color/image_raw` - Immagine RGB
- `/camera/depth/image_raw` - Immagine depth
- `/camera/points` - Point cloud

### 4. Avviare Web Interface

```bash
cd ~/MekoAiAccelerator

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Variabili
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8081

# Avvia web interface
python3 -m remote_ur_control.web_interface
```

**Accesso browser:**
- URL: `http://192.168.10.191:8081`
- Joystick virtuale per controllo robot
- Monitor ROS2 integrato

### 5. Test MuJoCo Viewer

```bash
# Avvia viewer MuJoCo
python3 -m mujoco.viewer \
    --mjcf ~/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

**Cosa vedi:**
- Simulazione 3D robot UR5e
- Controllo interattivo
- Visualizzazione fisica

### 6. Verificare Topic ROS2

```bash
# Lista tutti i topic
ros2 topic list

# Topic robot UR
ros2 topic echo /joint_states
ros2 topic echo /tf

# Topic camera
ros2 topic echo /camera/color/image_raw
ros2 topic echo /camera/depth/image_raw

# Topic comandi
ros2 topic echo /forward_velocity_controller/commands
```

### 7. Monitorare Sistema

```bash
# Processi ROS2
ros2 node list
ros2 topic list
ros2 service list

# Stato sistema
htop
nvidia-smi  # GPU Jetson

# Log ROS2
ros2 topic echo /rosout
```

---

## 🖥️ DESKTOP REMOTO (VNC)

### Configurare VNC (se non attivo)

**Installazione TigerVNC:**
```bash
sudo apt update
sudo apt install tigervnc-standalone-server tigervnc-common

# Configura password VNC
vncpasswd

# Avvia server VNC
vncserver :1 -geometry 1920x1080 -depth 24
```

**Avvio automatico:**
```bash
# Crea file ~/.vnc/xstartup
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
/etc/X11/xinit/xinitrc
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
x-window-manager &
EOF

chmod +x ~/.vnc/xstartup
```

**Connessione:**
- Client VNC → `192.168.10.191:5901`
- Username: `lab`
- Password: (quella configurata con vncpasswd)

---

## 📂 STRUTTURA FILE IMPORTANTI

```
~/MekoAiAccelerator/
├── remote_ur_control/
│   ├── web_interface.py          # Web interface joystick
│   ├── remote_ur_controller.py    # Controller remoto
│   └── rtde_teleop.py            # Teleoperazione RTDE
├── ros2_bridge_fixed.py           # Bridge ROS2
├── verifica_installazione_completa.py
└── *.md                           # Documentazione

~/ros2_ws/
├── src/
│   ├── Universal_Robots_ROS2_Driver/  # Driver UR
│   ├── OrbbecSDK_ROS2/                # Driver camera
│   └── remote_ur_control_ros2/        # Componenti custom
└── install/                           # Pacchetti compilati

~/mujoco_menagerie/
├── universal_robots_ur5e/        # Modello UR5e
└── universal_robots_ur10e/       # Modello UR10e
```

---

## 🎯 WORKFLOW TIPICO

### 1. Controllo Robot Remoto
```bash
# Terminale 1: Driver UR
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194

# Terminale 2: Web Interface
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export UR_ROBOT_IP=192.168.10.194
python3 -m remote_ur_control.web_interface

# Browser: http://192.168.10.191:8081
```

### 2. Vision + Detection
```bash
# Terminale 1: Camera
ros2 launch orbbec_camera gemini_330_series.launch.py

# Terminale 2: Object Detection (da implementare)
# ros2 run yolo_detector yolo_node

# Terminale 3: Visualizzazione
rviz2
```

### 3. Simulazione MuJoCo
```bash
python3 -m mujoco.viewer --mjcf ~/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

---

## 🔍 VERIFICA STATO SISTEMA

### Script di Verifica
```bash
cd ~/MekoAiAccelerator
python3 verifica_installazione_completa.py
```

### Comandi Rapidi
```bash
# ROS2 packages
ros2 pkg list | wc -l

# Processi attivi
ps aux | grep ros2

# Topic attivi
ros2 topic list

# Nodi attivi
ros2 node list
```

---

## 📞 SUPPORTO

### Problemi Comuni

1. **VNC non funziona**
   - Verifica: `systemctl status vncserver@*`
   - Riavvia: `vncserver -kill :1 && vncserver :1`

2. **ROS2 non trova pacchetti**
   - Source: `source ~/ros2_ws/install/setup.bash`

3. **Robot non risponde**
   - Verifica IP: `ping 192.168.10.194`
   - Verifica programma robot in PLAYING

4. **Camera non funziona**
   - Verifica connessione USB
   - Verifica permessi: `ls -l /dev/video*`

---

## ✅ CHECKLIST ACCESSO

- [ ] SSH funzionante: `ssh lab@192.168.10.191`
- [ ] VNC configurato (opzionale)
- [ ] ROS2 source funziona
- [ ] Driver UR compilato
- [ ] Web interface accessibile
- [ ] Camera Orbbec connessa (se disponibile)

**Tutto pronto per iniziare! 🚀**






