# Deploy Completo su AI Accelerator con Orbbec

Guida completa per il deploy del sistema di teleoperazione robot UR con camera Orbbec sull'AI Accelerator.

## Prerequisiti

- Accesso SSH all'AI Accelerator (192.168.10.191)
- Camera Orbbec collegata all'AI Accelerator
- Robot UR acceso e configurato

## Deploy Automatico

### Da Windows (Git Bash o WSL)

```bash
bash deploy_to_ai_accelerator_complete.sh
```

Lo script:
1. Trasferisce tutti i file necessari
2. Installa ROS2 Humble (se non presente)
3. Installa Universal Robots ROS2 Driver
4. Installa OrbbecSDK_ROS2
5. Crea pacchetto ROS2 personalizzato per teleoperazione
6. Configura tutto per funzionare insieme

## Setup Manuale

### 1. Connettiti all'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

### 2. Source ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 3. Verifica Installazioni

```bash
# Verifica ROS2
ros2 --version

# Verifica driver UR
ros2 pkg list | grep ur_robot_driver

# Verifica Orbbec
ros2 pkg list | grep orbbec

# Verifica teleop personalizzato
ros2 pkg list | grep meko_teleop
```

## Avvio Sistema Completo

### Terminale 1: Orbbec Camera

```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia camera Orbbec
ros2 launch orbbec_camera orbbec_camera.launch.py
```

Dovresti vedere:
```
[INFO] [orbbec_camera]: Camera initialized
[INFO] [orbbec_camera]: Publishing RGB and Depth topics
```

Verifica topic:
```bash
ros2 topic list | grep camera
# Dovresti vedere:
# /camera/color/image_raw
# /camera/depth/image_raw
# /camera/points
```

### Terminale 2: UR Robot Driver

```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia driver UR
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Sul Teach Pendant del robot:**
1. Avvia programma con **External Control**
2. IP Host: `192.168.10.191`
3. Porta: `50002`
4. Premi **PLAY**

Dovresti vedere:
```
[INFO] [ur_ros2_control_node]: Robot ready to receive control commands.
```

### Terminale 3: Teleoperazione con Orbbec

```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia nodo teleoperazione
ros2 run meko_teleop orbbec_teleop_node
```

### Terminale 4: Web Interface (Alternativa)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

python3 -m remote_ur_control.web_interface
```

Accedi da browser: `http://192.168.10.191:8080`

## Verifica Funzionamento

### 1. Verifica Topic Camera

```bash
# Vedi immagini RGB
ros2 topic echo /camera/color/image_raw --no-arr

# Vedi immagini depth
ros2 topic echo /camera/depth/image_raw --no-arr

# Vedi point cloud
ros2 topic echo /camera/points --no-arr
```

### 2. Verifica Topic Robot

```bash
# Lista topic robot
ros2 topic list | grep -E "(servo|joint|controller)"

# Monitora comandi velocità
ros2 topic echo /forward_velocity_controller/commands

# Monitora comandi twist
ros2 topic echo /servo_node/delta_twist_cmds
```

### 3. Test Movimento

Usa la web interface o il nodo teleoperazione per inviare comandi e verifica che:
- I comandi arrivino ai topic ROS2
- Il robot si muova correttamente
- La camera pubblichi dati

## Repository Utilizzati

1. **Universal Robots ROS2 Driver**
   - Repository: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
   - Branch: `humble`
   - Fornisce: Controllo robot UR via ROS2

2. **OrbbecSDK_ROS2**
   - Repository: https://github.com/orbbec/OrbbecSDK_ROS2
   - Fornisce: Integrazione camera Orbbec con ROS2
   - Topic: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/points`

3. **Meko Teleop** (Personalizzato)
   - Pacchetto creato durante deploy
   - Fornisce: Nodo ROS2 per teleoperazione con visione
   - Nodo: `orbbec_teleop_node`

## Sviluppo Teleoperazione

Il nodo `orbbec_teleop_node` è un template base. Puoi estenderlo per:

1. **Tracking oggetti**: Usa OpenCV per tracciare oggetti nell'immagine RGB
2. **Controllo posizione**: Usa depth per controllare distanza dal target
3. **Hand-eye coordination**: Combina visione e controllo robot
4. **AI/ML**: Integra modelli di deep learning per riconoscimento/controllo

### Esempio Estensione

Modifica `~/ros2_ws/src/meko_teleop/meko_teleop/orbbec_teleop_node.py`:

```python
def process_and_control(self):
    """Elabora dati camera e genera comandi robot."""
    if self.current_rgb is not None and self.current_depth is not None:
        # Esempio: tracking centro immagine
        h, w = self.current_rgb.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # Leggi depth al centro
        depth_value = self.current_depth[center_y, center_x]
        
        # Genera comando twist basato su posizione
        twist = Twist()
        if depth_value > 0:
            # Controlla movimento basato su depth
            twist.linear.x = 0.01  # Avanti lento
            self.twist_pub.publish(twist)
```

Dopo modifiche, ricompila:
```bash
cd ~/ros2_ws
colcon build --packages-select meko_teleop
source install/setup.bash
```

## Troubleshooting

### Camera Orbbec non rilevata

```bash
# Verifica connessione USB
lsusb | grep Orbbec

# Verifica permessi
sudo chmod 666 /dev/bus/usb/*/*

# Riavvia nodo camera
ros2 launch orbbec_camera orbbec_camera.launch.py
```

### Robot non risponde

1. Verifica connessione robot:
   ```bash
   ping 192.168.10.194
   ```

2. Verifica che programma sul teach pendant sia in PLAYING

3. Verifica topic robot:
   ```bash
   ros2 topic list | grep ur
   ```

### ROS2 non trovato

```bash
# Installa ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build errors

```bash
# Aggiorna dipendenze
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Ricompila
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Riferimenti

- [Universal Robots ROS2 Driver Docs](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Orbbec ROS2 SDK](https://github.com/orbbec/OrbbecSDK_ROS2)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

