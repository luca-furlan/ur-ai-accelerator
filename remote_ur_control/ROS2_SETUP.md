# ROS2 Setup per Controllo Joystick UR

## Perché ROS2?

Il controllo joystick fluido richiede:
- **125Hz real-time communication** (8ms cycle)
- **Servo mode** con `servoj` invece di `speedj`/`speedl`
- **Hardware interface** collaudato

Il socket URScript (porta 30002) **non è adatto** per controllo real-time joystick perché:
- Buffer limitato → scatti
- Frequenza massima ~5-10Hz → non fluido
- No feedback posizione in tempo reale

## Soluzione: Universal Robots ROS2 Driver

Repository ufficiale: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

### Vantaggi
✅ **Servo mode** a 125Hz testato e stabile
✅ **Twist controller** per joystick cartesiano 6D
✅ **RTDE** per feedback real-time
✅ **Collaudato** da migliaia di utenti

## Installazione su AI Accelerator (Ubuntu 22.04 ARM64)

### 1. Verifica ROS2 Humble

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

Se non installato:
```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

### 2. Crea Workspace ROS2

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clona Universal Robots Driver

```bash
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
```

### 4. Installa Dipendenze

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build

```bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 6. Configura Robot UR

Sul **Teach Pendant**:
1. **Program** → **URCaps** → Installa **External Control**
2. Crea programma con nodo **External Control**
3. IP Host: `192.168.10.191` (IP AI Accelerator)
4. Porta: `50002` (default)
5. Salva programma come `ros_control.urp`

### 7. Test Connessione

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Sul Teach Pendant:** Play del programma `ros_control.urp`

Dovresti vedere:
```
[ur_ros2_control_node]: Robot ready to receive control commands.
```

### 8. Servo Mode per Joystick

In un altro terminale:

```bash
source ~/ros2_ws/install/setup.bash

# Attiva servo mode
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger

# Test twist (velocità cartesiana)
ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "
  twist:
    linear: {x: 0.0, y: 0.05, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}"
```

Il robot dovrebbe muoversi **fluido** a 50mm/s in direzione Y!

## Prossimi Passi

1. ✅ Installare ROS2 + UR driver
2. ⏳ Creare web interface che pubblica su `/servo_node/delta_twist_cmds`
3. ⏳ Usare `rosbridge_suite` per WebSocket → ROS2
4. ⏳ Joystick HTML5 → WebSocket → ROS2 topic

## Riferimenti

- [UR ROS2 Driver Docs](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
- [Servo Control](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/doc/usage.rst#using-the-servo-node)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

