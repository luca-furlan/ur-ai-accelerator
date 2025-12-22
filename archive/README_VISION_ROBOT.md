# Sistema Vision + Robot Integrato

Sistema completo per controllo robot UR con visione artificiale usando camera Orbecc e YOLOv8.

## 🎯 Architettura

```
Hardware Reale (Robot UR + Camera Orbecc)
           ↓
ROS2 Humble + MoveIt2
           ↓
Camera Orbecc → YOLOv8 Detection → Motion Planning → Robot Control
```

## 📦 Componenti

### 1. **Vision YOLO Detector** (`vision_yolo_detector.py`)
- Sottoscrive ai topic della camera Orbecc (RGB + Depth)
- Esegue object detection con YOLOv8
- Calcola coordinate 3D degli oggetti rilevati usando depth
- Pubblica detections su topic ROS2

### 2. **MoveIt Vision Controller** (`moveit_vision_controller.py`)
- Riceve detections 3D
- Seleziona target basato su priorità e sicurezza
- Coordina con MoveIt2 per planning movimenti
- Pubblica target selezionati

### 3. **Vision Robot Coordinator** (`vision_robot_coordinator.py`)
- Coordinatore centrale del sistema
- Gestisce stato e sicurezza
- Interfaccia diretta con robot UR
- Esegue sequenze pick & place

### 4. **Vision Web API** (`remote_ur_control/vision_web_api.py`)
- Estensione API REST per web interface
- Endpoints per controllo vision system
- Integrazione con web interface esistente

## 🚀 Avvio Rapido

### Prerequisiti

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Python packages
pip install ultralytics opencv-python numpy

# ROS2 packages
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs
```

### Installazione Camera Orbecc

```bash
# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Build
cd ~/ros2_ws
colcon build --packages-select orbbec_camera

# Source
source install/setup.bash
```

### Avvio Sistema Completo

**Metodo 1: Script automatico (raccomandato)**

```bash
# Avvia tutto
./avvia_vision_robot_system.sh

# Con parametri personalizzati
export ROBOT_IP=192.168.10.194
export AUTO_MODE=false
export YOLO_MODEL=yolov8n.pt
./avvia_vision_robot_system.sh

# Ferma tutto
./ferma_vision_robot_system.sh
```

**Metodo 2: Launch file ROS2**

```bash
# Launch completo
ros2 launch launch_vision_robot_system.py \
    robot_ip:=192.168.10.194 \
    auto_mode:=false \
    yolo_model:=yolov8n.pt \
    confidence:=0.5

# Solo detection (senza controllo robot)
ros2 launch launch_vision_robot_system.py \
    enable_robot_control:=false
```

**Metodo 3: Nodi singoli (per debug)**

```bash
# Terminal 1: Camera
ros2 launch orbecc_camera gemini_330_series.launch.py

# Terminal 2: Vision Detector
python3 vision_yolo_detector.py

# Terminal 3: MoveIt Controller
python3 moveit_vision_controller.py

# Terminal 4: Coordinator
python3 vision_robot_coordinator.py
```

## 📡 ROS2 Topics

### Subscribed (Input)

| Topic | Type | Descrizione |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Feed RGB camera |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Feed depth camera |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics camera |

### Published (Output)

| Topic | Type | Descrizione |
|-------|------|-------------|
| `/vision/detections` | `vision_msgs/Detection2DArray` | Detections 2D (standard ROS) |
| `/vision/detections_3d` | `std_msgs/String` (JSON) | Detections 3D con coordinate |
| `/vision/annotated_image` | `sensor_msgs/Image` | Immagine con bounding boxes |
| `/vision/selected_target` | `geometry_msgs/PoseStamped` | Target selezionato per robot |
| `/vision/system_status` | `std_msgs/String` (JSON) | Stato sistema |

### Command Topics

| Topic | Type | Descrizione |
|-------|------|-------------|
| `/vision/coordinator_command` | `std_msgs/String` (JSON) | Comandi al coordinator |
| `/vision/coordinator_response` | `std_msgs/String` (JSON) | Risposte dal coordinator |

## 🎮 Comandi

### Via ROS2 Topic

```bash
# Pick target selezionato
ros2 topic pub --once /vision/coordinator_command std_msgs/String \
    '{data: "{\"command\": \"pick_target\"}"}'

# Stop robot
ros2 topic pub --once /vision/coordinator_command std_msgs/String \
    '{data: "{\"command\": \"stop\"}"}'

# Pause sistema
ros2 topic pub --once /vision/coordinator_command std_msgs/String \
    '{data: "{\"command\": \"pause\"}"}'

# Resume
ros2 topic pub --once /vision/coordinator_command std_msgs/String \
    '{data: "{\"command\": \"resume\"}"}'

# Reset
ros2 topic pub --once /vision/coordinator_command std_msgs/String \
    '{data: "{\"command\": \"reset\"}"}'
```

### Via Web API (se integrato)

```bash
# Status sistema
curl http://localhost:5000/api/vision/status

# Detections correnti
curl http://localhost:5000/api/vision/detections

# Pick oggetto
curl -X POST http://localhost:5000/api/vision/pick_target \
    -H "Content-Type: application/json" \
    -d '{"class": "bottle"}'

# Seleziona classe target
curl -X POST http://localhost:5000/api/vision/select_class \
    -H "Content-Type: application/json" \
    -d '{"class": "cup"}'
```

## 🔧 Configurazione

### Parametri Vision Detector

```yaml
camera_rgb_topic: '/camera/color/image_raw'
camera_depth_topic: '/camera/depth/image_raw'
camera_info_topic: '/camera/color/camera_info'
yolo_model: 'yolov8n.pt'  # n=nano, s=small, m=medium, l=large, x=xlarge
confidence_threshold: 0.5
publish_rate: 10.0  # Hz
enable_visualization: true
```

### Parametri MoveIt Controller

```yaml
planning_group: 'ur_manipulator'
end_effector_link: 'tool0'
reference_frame: 'base_link'
target_classes: ['bottle', 'cup', 'cell phone', 'person']
approach_offset: 0.15  # metri sopra oggetto
auto_mode: false
safety_distance: 0.1  # metri
```

### Parametri Coordinator

```yaml
robot_ip: '192.168.10.194'
enable_robot_control: true
safe_mode: true
auto_pick: false
```

## 📊 Visualizzazione

### RViz

```bash
# Visualizza immagine annotata
ros2 run rqt_image_view rqt_image_view /vision/annotated_image

# Oppure con RViz
rviz2
# Aggiungi display: Image → /vision/annotated_image
```

### Monitor Topics

```bash
# Detections in tempo reale
ros2 topic echo /vision/detections_3d

# Status sistema
ros2 topic echo /vision/system_status

# Target selezionato
ros2 topic echo /vision/selected_target
```

## 🔍 Debug

### Verifica Camera

```bash
# Lista topics camera
ros2 topic list | grep camera

# Test image feed
ros2 topic hz /camera/color/image_raw

# Visualizza immagine
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

### Verifica Detection

```bash
# Test YOLOv8
python3 -c "from ultralytics import YOLO; m = YOLO('yolov8n.pt'); print('OK')"

# Verifica detection topic
ros2 topic hz /vision/detections_3d

# Log detector
ros2 node info /vision_yolo_detector
```

### Verifica Robot Connection

```bash
# Test ping
ping 192.168.10.194

# Test RTDE connection
python3 -c "import rtde_control; rt = rtde_control.RTDEControlInterface('192.168.10.194'); print('Connected')"
```

### Logs

```bash
# Logs nodi ROS2
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# Logs sistema (se avviato con script)
tail -f /tmp/vision_detector.log
tail -f /tmp/moveit_controller.log
tail -f /tmp/coordinator.log
```

## 🛡️ Sicurezza

### Safe Mode

Quando `safe_mode: true`:
- Velocità limitata (max 0.1 m/s)
- Accelerazione limitata (max 0.05 m/s²)
- Verifica distanza minima oggetti
- Stop automatico in caso errori

### Limiti

```python
# Distanze sicurezza
MIN_DISTANCE = 0.1  # metri
MAX_DISTANCE = 2.0  # metri

# Velocità
MAX_SPEED = 0.2  # m/s in safe mode
MAX_ACCELERATION = 0.1  # m/s²

# Workspace limits (da configurare)
X_MIN, X_MAX = -1.0, 1.0
Y_MIN, Y_MAX = -1.0, 1.0
Z_MIN, Z_MAX = 0.0, 1.5
```

## 🔗 Integrazione Web Interface

### Aggiungi Vision API a Web Interface Esistente

```python
# In web_interface.py
from remote_ur_control.vision_web_api import add_vision_routes_to_app

# Dopo creazione Flask app
app = Flask(__name__)
vision_api = add_vision_routes_to_app(app)

# Ora disponibili nuovi endpoints:
# GET  /api/vision/status
# GET  /api/vision/detections
# POST /api/vision/pick_target
# POST /api/vision/select_class
```

## 🎯 Use Cases

### 1. Pick & Place Automatico

```bash
# Avvia in auto mode
export AUTO_MODE=true
./avvia_vision_robot_system.sh

# Il robot rileverà automaticamente oggetti target e si muoverà verso di essi
```

### 2. Pick Manuale con Selezione

```bash
# Avvia in manual mode
export AUTO_MODE=false
./avvia_vision_robot_system.sh

# Seleziona classe e pick
curl -X POST http://localhost:5000/api/vision/select_class \
    -d '{"class": "bottle"}'

curl -X POST http://localhost:5000/api/vision/pick_target
```

### 3. Solo Detection (no Robot)

```bash
# Usa solo per rilevamento oggetti
ros2 launch launch_vision_robot_system.py \
    enable_robot_control:=false

# Visualizza detections
ros2 topic echo /vision/detections_3d
```

## 📝 TODO / Miglioramenti Futuri

- [ ] Implementare TF2 transformation per camera → base robot
- [ ] Aggiungere MoveIt2 motion planning completo
- [ ] Implementare gripper control
- [ ] Aggiungere collision detection
- [ ] Implementare trajectory smoothing
- [ ] Aggiungere calibrazione camera-robot automatica
- [ ] Implementare multi-object tracking
- [ ] Aggiungere web UI per visualizzazione
- [ ] Implementare logging e replay
- [ ] Aggiungere support per custom YOLO models

## 🐛 Troubleshooting

### Camera non trovata

```bash
# Verifica USB connection
lsusb | grep Orbbec

# Permissions
sudo chmod 666 /dev/video*

# Reinstalla driver
cd ~/ros2_ws
colcon build --packages-select orbbec_camera --cmake-clean-cache
```

### YOLO troppo lento

```bash
# Usa model più piccolo
export YOLO_MODEL=yolov8n.pt  # nano (più veloce)

# Riduci risoluzione camera
# (modifica launch file camera)
```

### Robot non si muove

```bash
# Verifica connessione
ping 192.168.10.194

# Verifica robot stato
python3 remote_ur_control/check_robot_status.py

# Verifica controller attivo
ros2 topic list | grep scaled_joint_trajectory_controller
```

## 📚 Riferimenti

- **YOLOv8**: https://docs.ultralytics.com/
- **Orbecc Camera**: https://github.com/orbbec/OrbbecSDK_ROS2
- **MoveIt2**: https://moveit.picknik.ai/humble/index.html
- **ROS2 Humble**: https://docs.ros.org/en/humble/

## 📧 Supporto

Per problemi o domande, consulta la documentazione o apri una issue.




