# Controllo ROS2 per UR Robot

## Setup ROS2

**IMPORTANTE**: Il controllo fluido richiede ROS2 con il driver UR ufficiale.

### 1. Avvia ROS2 Driver sul AI Accelerator

```bash
# SSH su AI Accelerator (192.168.10.191)
ssh lab@192.168.10.191

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # se hai installato il driver

# Avvia driver UR
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

### 2. Sul Teach Pendant

1. Crea programma con **External Control** URCap
2. IP Host: `192.168.10.191` (AI Accelerator)
3. Porta: `50002`
4. Salva come `ros_control.urp`
5. **PLAY** il programma

### 3. Verifica ROS2

```bash
# Su AI Accelerator
ros2 topic list
# Dovresti vedere:
# /servo_node/delta_twist_cmds
# /joint_states
# etc.
```

### 4. Web Interface

La web interface ora usa ROS2 automaticamente se disponibile:
- Pubblica su `/servo_node/delta_twist_cmds` per controllo fluido
- Fallback a socket se ROS2 non disponibile

## Vantaggi ROS2

✅ **125Hz real-time** - controllo fluido garantito
✅ **Servo node** - movimento senza scatti
✅ **Driver ufficiale** - testato e stabile
✅ **Nessun accumulo** - gestione corretta dei buffer

## Troubleshooting

**ROS2 non disponibile?**
- Verifica che ROS2 sia installato: `ros2 --version`
- Verifica che il driver sia compilato: `ros2 pkg list | grep ur_robot_driver`
- La web interface userà fallback socket (meno fluido)

**Robot non si muove?**
- Verifica che il programma sul teach pendant sia PLAYING
- Verifica che ROS2 driver sia connesso: `ros2 topic echo /joint_states`
- Controlla errori nella console ROS2


