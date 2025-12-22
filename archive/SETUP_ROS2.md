# Setup ROS2 per Controllo UR Robot

## 1. Verifica ROS2 sull'AI Accelerator

```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
ros2 --version
```

Se non installato:
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

## 2. Installa Universal Robots ROS2 Driver

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
cd ~/ros2_ws

# Installa dipendenze
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. Configura Robot sul Teach Pendant

1. **Program** → **URCaps** → Installa **External Control**
2. Crea programma con nodo **External Control**
3. IP Host: `192.168.10.191` (AI Accelerator)
4. Porta: `50002`
5. Salva come `ros_control.urp`

## 4. Avvia ROS2 Driver

**Terminale 1:**
```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Sul Teach Pendant**: **PLAY** del programma `ros_control.urp`

Dovresti vedere:
```
[ur_ros2_control_node]: Robot ready to receive control commands.
```

## 5. Verifica Topic ROS2

**Terminale 2:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Lista topic
ros2 topic list | grep servo

# Dovresti vedere:
# /servo_node/delta_twist_cmds
# /servo_node/delta_joint_cmds

# Monitora comandi
ros2 topic echo /servo_node/delta_twist_cmds
```

## 6. Avvia Web Interface

**Terminale 3:**
```bash
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

python3 -m remote_ur_control.web_interface
```

Dovresti vedere:
```
✅ ROS2 bridge initialized
 * Running on http://192.168.10.191:8080
```

## 7. Testa

Apri browser: `http://192.168.10.191:8080`

Muovi il joystick → vedi messaggi in Terminale 2 (`ros2 topic echo`)

## Troubleshooting

**"ROS2 bridge not available"?**
- Verifica: `source /opt/ros/humble/setup.bash`
- Verifica: `python3 -c "import rclpy"`
- Se manca rclpy: `pip3 install --user rclpy`

**"Failed to initialize ROS2"?**
- Verifica che ROS2 driver sia in esecuzione
- Verifica che robot sia in modalità External Control
- Controlla errori nella console ROS2

**Robot non si muove?**
- Verifica che programma sul teach pendant sia PLAYING
- Verifica topic: `ros2 topic echo /servo_node/delta_twist_cmds`
- Controlla che comandi arrivino dal browser


