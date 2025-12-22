# 🚀 Quick Start - Sistema Vision + Robot

Guida rapida per avviare il sistema completo Camera Orbecc → YOLOv8 → Robot UR.

## 📋 Prerequisiti (5 minuti)

### 1. Installa dipendenze

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Python packages
pip install ultralytics opencv-python numpy

# ROS2 packages (se non già installati)
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-moveit
```

### 2. Installa driver camera Orbecc

```bash
# Clone repo
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Build
cd ~/ros2_ws
colcon build --packages-select orbecc_camera

# Source
source install/setup.bash
```

## ⚡ Avvio Ultra-Rapido (1 comando)

```bash
cd /path/to/MekoAiAccelerator
./avvia_vision_robot_system.sh
```

**Fatto!** Il sistema è ora attivo.

## 🎮 Test Sistema

### Verifica tutto funziona

```bash
python3 test_vision_system.py
```

Dovresti vedere:
```
✅ Camera Orbecc         OK
✅ YOLOv8                OK  
✅ ROS2                  OK
✅ Vision Nodes          OK
✅ Robot Connection      OK
```

### Visualizza detections

```bash
# Stream video annotato
ros2 run rqt_image_view rqt_image_view /vision/annotated_image

# Detections in JSON
ros2 topic echo /vision/detections_3d
```

### Monitor sistema

```bash
# Status tempo reale
ros2 topic echo /vision/system_status

# Lista nodi attivi
ros2 node list | grep vision
```

## 🤖 Controllo Robot

### Modalità Manuale (sicura)

```bash
# Avvia in manual mode (default)
./avvia_vision_robot_system.sh

# In altro terminale: seleziona oggetto e pick
curl -X POST http://localhost:5000/api/vision/select_class \
    -H "Content-Type: application/json" \
    -d '{"class": "bottle"}'

curl -X POST http://localhost:5000/api/vision/pick_target
```

### Modalità Automatica (avanzata)

```bash
# ATTENZIONE: robot si muove automaticamente verso oggetti rilevati!
export AUTO_MODE=true
./avvia_vision_robot_system.sh
```

### Stop Emergenza

```bash
# Stop robot
ros2 topic pub --once /vision/coordinator_command std_msgs/String \
    '{data: "{\"command\": \"stop\"}"}'

# Oppure ferma tutto
./ferma_vision_robot_system.sh
```

## 🌐 Web Interface

### Integra Vision API nella web interface esistente

```bash
# Integra automaticamente
python3 integra_vision_web_interface.py

# Avvia web interface
python -m remote_ur_control.web_interface
```

### Nuovi endpoint disponibili

```bash
# Status vision system
curl http://localhost:5000/api/vision/status

# Detections correnti  
curl http://localhost:5000/api/vision/detections

# Pick oggetto
curl -X POST http://localhost:5000/api/vision/pick_target \
    -d '{"class": "cup"}'
```

## 🎯 Use Cases Comuni

### 1. Pick & Place di Bottiglia

```bash
# Terminal 1: Avvia sistema
./avvia_vision_robot_system.sh

# Terminal 2: Quando vedi la bottiglia
curl -X POST localhost:5000/api/vision/select_class -d '{"class":"bottle"}'
curl -X POST localhost:5000/api/vision/pick_target

# Robot si muoverà verso la bottiglia rilevata
```

### 2. Solo Detection (no movimento robot)

```bash
# Avvia senza controllo robot
ros2 launch launch_vision_robot_system.py \
    enable_robot_control:=false

# Visualizza detections
ros2 topic echo /vision/detections_3d
```

### 3. Detection Custom con YOLO Large

```bash
# Usa modello YOLO più grande (più accurato, più lento)
export YOLO_MODEL=yolov8l.pt
./avvia_vision_robot_system.sh
```

## 🔧 Configurazione Rapida

### Cambia IP Robot

```bash
export ROBOT_IP=192.168.1.100
./avvia_vision_robot_system.sh
```

### Cambia confidence threshold

```bash
export CONFIDENCE=0.7  # Default 0.5
./avvia_vision_robot_system.sh
```

### Cambia modello YOLO

```bash
# Opzioni: yolov8n.pt (nano), yolov8s.pt (small), yolov8m.pt (medium)
#          yolov8l.pt (large), yolov8x.pt (xlarge)
export YOLO_MODEL=yolov8s.pt
./avvia_vision_robot_system.sh
```

## 📊 Monitoraggio

### Dashboard veloce

```bash
# Terminal 1: Status
watch -n 1 'ros2 topic echo /vision/system_status --once'

# Terminal 2: Detections
watch -n 1 'ros2 topic echo /vision/detections_3d --once | head -20'

# Terminal 3: Visualizza immagine
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

### Logs

```bash
# Logs nodi (se avviato con script)
tail -f /tmp/vision_detector.log
tail -f /tmp/moveit_controller.log  
tail -f /tmp/coordinator.log
```

## 🐛 Troubleshooting Veloce

### Camera non trovata

```bash
# Verifica USB
lsusb | grep Orbbec

# Rilancia camera
ros2 launch orbecc_camera gemini_330_series.launch.py
```

### YOLO non trova model

```bash
# Download model manualmente
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### Robot non risponde

```bash
# Verifica ping
ping 192.168.10.194

# Check robot status
python3 remote_ur_control/check_robot_status.py

# Verifica controller
ros2 control list_controllers
```

### Vision nodes non partono

```bash
# Verifica ROS2 environment
printenv | grep ROS

# Re-source
source /opt/ros/humble/setup.bash

# Riavvia
./ferma_vision_robot_system.sh
./avvia_vision_robot_system.sh
```

## 📚 Documentazione Completa

Vedi `README_VISION_ROBOT.md` per documentazione dettagliata.

## 🎓 Tutorial Step-by-Step

### Tutorial 1: Prima detection

```bash
# 1. Avvia sistema
./avvia_vision_robot_system.sh

# 2. Attendi caricamento (10-15 sec)

# 3. In altro terminale: monitor detections
ros2 topic echo /vision/detections_3d

# 4. Metti un oggetto davanti alla camera (bottiglia, tazza, telefono)

# 5. Dovresti vedere JSON con oggetto rilevato!
```

### Tutorial 2: Primo movimento robot

```bash
# 1. Avvia sistema
./avvia_vision_robot_system.sh

# 2. Metti bottiglia davanti camera

# 3. Quando rilevata, pick
curl -X POST localhost:5000/api/vision/select_class -d '{"class":"bottle"}'
curl -X POST localhost:5000/api/vision/pick_target

# 4. Robot si muove verso bottiglia!
```

### Tutorial 3: Visualizzazione

```bash
# 1. Avvia sistema
./avvia_vision_robot_system.sh

# 2. Apri RViz o rqt_image_view
ros2 run rqt_image_view rqt_image_view /vision/annotated_image

# 3. Vedrai video con bounding boxes YOLO in tempo reale!
```

## 🚀 Pro Tips

1. **Performance**: Usa `yolov8n.pt` (nano) per max speed, `yolov8l.pt` per max accuracy
2. **Lighting**: Camera funziona meglio con buona illuminazione
3. **Distance**: Oggetti rilevabili tra 0.3m e 2m dalla camera
4. **Safety**: Prima volta, usa `AUTO_MODE=false` per controllo manuale
5. **Debug**: Usa `ros2 topic hz` per verificare frequenza pubblicazione

## ⚡ Comandi Rapidi

```bash
# Avvio completo
./avvia_vision_robot_system.sh

# Test sistema
python3 test_vision_system.py

# Stop tutto
./ferma_vision_robot_system.sh

# Solo detection (no robot)
ros2 launch launch_vision_robot_system.py enable_robot_control:=false

# Monitor detections
ros2 topic echo /vision/detections_3d

# Visualizza video
ros2 run rqt_image_view rqt_image_view /vision/annotated_image

# Pick manuale
curl -X POST localhost:5000/api/vision/pick_target

# Stop robot
ros2 topic pub --once /vision/coordinator_command std_msgs/String '{data: "{\"command\": \"stop\"}"}'
```

## 📞 Supporto

- Documentazione completa: `README_VISION_ROBOT.md`
- Test sistema: `python3 test_vision_system.py`
- Logs: `/tmp/*.log`

---

**Ready!** Ora hai il sistema vision + robot completamente funzionante! 🎉




