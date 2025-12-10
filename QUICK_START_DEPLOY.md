# Quick Start - Deploy su AI Accelerator

## Deploy Rapido (1 comando)

Da Windows (Git Bash o WSL):

```bash
bash deploy_to_ai_accelerator_complete.sh
```

## Cosa viene installato

1. ✅ **ROS2 Humble** (se non presente)
2. ✅ **Universal Robots ROS2 Driver** - Controllo robot UR
3. ✅ **OrbbecSDK_ROS2** - Integrazione camera Orbbec
4. ✅ **Meko Teleop Package** - Nodo ROS2 personalizzato per teleoperazione
5. ✅ **Web Interface** - Interfaccia web per controllo robot

## Dopo il Deploy

### 1. Connettiti all'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

### 2. Avvia Sistema Completo

**Terminale 1 - Camera Orbbec:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch orbbec_camera orbbec_camera.launch.py
```

**Terminale 2 - Robot UR:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194
```

**Sul Teach Pendant:** Avvia programma con External Control (IP: 192.168.10.191, Porta: 50002)

**Terminale 3 - Teleoperazione:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run meko_teleop orbbec_teleop_node
```

**Oppure Web Interface:**
```bash
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
python3 -m remote_ur_control.web_interface
```

Apri browser: `http://192.168.10.191:8080`

## Repository Utilizzati

- **Universal Robots ROS2 Driver**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **OrbbecSDK_ROS2**: https://github.com/orbbec/OrbbecSDK_ROS2
- **Meko Teleop**: Pacchetto personalizzato creato durante deploy

## Documentazione Completa

Vedi `DEPLOY_COMPLETE.md` per dettagli completi.

