# 🚀 Come Usare Sistema Vision

## 📍 Dove Sei Ora

Hai già la web interface funzionante che avviavi così:

```bash
lab@ubuntu:~/MekoAiAccelerator$ ./avvia_web_interface_joystick.sh
```

Accessibile su: `http://192.168.10.191:8080/`

## ✨ Cosa Aggiungiamo

Sistema vision con:
- **Camera Orbecc** - RGB + Depth
- **YOLOv8** - Object detection real-time
- **Coordinate 3D** - Posizione oggetti nello spazio
- **Pick automatico** - Robot si muove verso oggetti rilevati

## 🎯 Due Modi per Usarlo

### **Opzione 1: Solo Vision (web interface normale + API vision)**

Usa lo script esistente + chiama API vision da browser/codice

```bash
# Avvia come sempre
lab@ubuntu:~/MekoAiAccelerator$ ./avvia_web_interface_joystick.sh

# Da altro terminale: avvia solo vision nodes
lab@ubuntu:~/MekoAiAccelerator$ bash avvia_vision_robot_system.sh
```

Poi usa API REST:
```bash
# Vedi detections
curl http://192.168.10.191:8080/api/vision/detections

# Pick oggetto
curl -X POST http://192.168.10.191:8080/api/vision/pick_target
```

### **Opzione 2: Tutto Integrato (RACCOMANDATO)**

Un solo comando che avvia tutto insieme:

```bash
lab@ubuntu:~/MekoAiAccelerator$ ./avvia_web_interface_con_vision.sh
```

Questo avvia:
- ✅ Camera Orbecc
- ✅ Vision YOLO Detector
- ✅ MoveIt Controller
- ✅ Robot Coordinator
- ✅ Web Interface (porta 8080)

## 📋 Setup Iniziale (solo prima volta)

### 1. Deploy file su AI Accelerator

**Da Windows (se usi Git Bash/WSL):**
```bash
export AI_ACCELERATOR_IP=192.168.10.191
export AI_ACCELERATOR_USER=lab
bash deploy_vision_completo.sh
```

**Oppure manualmente (WinSCP/scp):**
```bash
# Copia questi file su AI Accelerator in ~/MekoAiAccelerator/
scp vision_*.py lab@192.168.10.191:~/MekoAiAccelerator/
scp moveit_*.py lab@192.168.10.191:~/MekoAiAccelerator/
scp launch_vision_robot_system.py lab@192.168.10.191:~/MekoAiAccelerator/
scp avvia_web_interface_con_vision.sh lab@192.168.10.191:~/MekoAiAccelerator/
scp integra_vision_web_interface.py lab@192.168.10.191:~/MekoAiAccelerator/
scp -r remote_ur_control/vision_web_api.py lab@192.168.10.191:~/MekoAiAccelerator/remote_ur_control/
```

### 2. Installa dipendenze

**Su AI Accelerator:**
```bash
ssh lab@192.168.10.191

cd ~/MekoAiAccelerator

# Python packages
pip3 install ultralytics opencv-python numpy --user

# ROS2 packages (se non già installati)
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs

# Camera Orbecc driver (se non già installato)
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd ~/ros2_ws
colcon build --packages-select orbbec_camera
source install/setup.bash
```

### 3. Rendi eseguibili

```bash
cd ~/MekoAiAccelerator
chmod +x *.sh *.py
```

### 4. Test

```bash
python3 test_vision_system.py
```

## 🚀 Utilizzo Quotidiano

### Avvio Sistema

```bash
lab@ubuntu:~$ cd ~/MekoAiAccelerator
lab@ubuntu:~/MekoAiAccelerator$ ./avvia_web_interface_con_vision.sh
```

**Output atteso:**
```
==========================================
🤖 AVVIO WEB INTERFACE + VISION SYSTEM
==========================================

[1/5] Configurazione ROS2...
  ✅ ROS2 Humble
  ✅ ROS2 Workspace

[2/5] Verifica dipendenze vision...
  ✅ YOLOv8 installato
  ✅ OpenCV installato
  ✅ cv_bridge disponibile

[3/5] Pulizia processi esistenti...
  ✅ Web interface terminata
  ✅ Vision nodes terminati

[4/5] Setup Vision API...
  ✅ Vision API già integrata

[5/5] Avvio componenti...

→ Verifica camera Orbecc...
  ✅ Camera avviata (PID: 12345)
→ Avvio Vision YOLO Detector...
  ✅ Detector avviato (PID: 12346)
→ Avvio MoveIt Vision Controller...
  ✅ Controller avviato (PID: 12347)
→ Avvio Vision Robot Coordinator...
  ✅ Coordinator avviato (PID: 12348)

==========================================
✅ SISTEMA ONLINE!
==========================================

Accedi da browser:
  http://192.168.10.191:8080

Componenti attivi:
  📷 Camera Orbecc (PID: 12345)
  👁️  Vision YOLO Detector (PID: 12346)
  🎯 MoveIt Controller (PID: 12347)
  🤖 Vision Coordinator (PID: 12348)

🚀 Avvio web interface...
```

### Uso da Browser

Apri browser su Windows:
```
http://192.168.10.191:8080
```

**Nuove funzionalità disponibili:**

1. **Status Vision System**
   ```javascript
   fetch('/api/vision/status')
   ```

2. **Vedi Detections**
   ```javascript
   fetch('/api/vision/detections')
   ```

3. **Seleziona Oggetto**
   ```javascript
   fetch('/api/vision/select_class', {
     method: 'POST',
     headers: {'Content-Type': 'application/json'},
     body: JSON.stringify({class: 'bottle'})
   })
   ```

4. **Pick Oggetto**
   ```javascript
   fetch('/api/vision/pick_target', {method: 'POST'})
   ```

### Stop Sistema

Premi **Ctrl+C** nel terminale dove gira lo script.

Oppure:
```bash
bash ferma_sistema_completo.sh
```

## 🎮 Workflow Tipico

### Scenario: Pick Bottiglia

1. **Avvia sistema**
   ```bash
   ./avvia_web_interface_con_vision.sh
   ```

2. **Apri browser**
   ```
   http://192.168.10.191:8080
   ```

3. **Metti bottiglia davanti camera**

4. **In browser (JavaScript console o Postman):**
   ```javascript
   // Vedi cosa rileva
   let res = await fetch('/api/vision/detections');
   let data = await res.json();
   console.log(data);
   
   // Se vedi bottiglia, selezionala
   await fetch('/api/vision/select_class', {
     method: 'POST',
     headers: {'Content-Type': 'application/json'},
     body: JSON.stringify({class: 'bottle'})
   });
   
   // Pick
   await fetch('/api/vision/pick_target', {method: 'POST'});
   ```

5. **Robot si muove verso bottiglia!**

## 📊 Monitoring

### Log in tempo reale

```bash
# Detector
tail -f /tmp/vision_detector.log

# Controller
tail -f /tmp/moveit_controller.log

# Coordinator
tail -f /tmp/coordinator.log
```

### ROS2 Topics

```bash
# Detections
ros2 topic echo /vision/detections_3d

# Status sistema
ros2 topic echo /vision/system_status

# Immagine annotata
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

## 🔧 Troubleshooting

### Camera non funziona

```bash
# Verifica USB
lsusb | grep Orbbec

# Verifica topics
ros2 topic list | grep camera

# Riavvia camera
pkill -f orbecc_camera
ros2 launch orbbec_camera gemini_330_series.launch.py
```

### Vision nodes non partono

```bash
# Verifica log
tail -f /tmp/vision_detector.log

# Verifica dipendenze
python3 -c "from ultralytics import YOLO"
python3 -c "from cv_bridge import CvBridge"
```

### Web interface non accessibile

```bash
# Verifica porta
netstat -tlnp | grep 8080

# Verifica processi
pgrep -af web_interface
```

## 📚 File Importanti

### Script avvio

- `avvia_web_interface_joystick.sh` - Web interface originale (come prima)
- `avvia_web_interface_con_vision.sh` - **NUOVO** - Web interface + vision
- `avvia_vision_robot_system.sh` - Solo vision nodes (senza web)

### Nodi Vision

- `vision_yolo_detector.py` - Detection YOLOv8
- `moveit_vision_controller.py` - Motion planning
- `vision_robot_coordinator.py` - Coordinamento robot

### Web

- `remote_ur_control/web_interface.py` - Web interface principale
- `remote_ur_control/vision_web_api.py` - API vision (integrata automaticamente)

## 💡 Tips

1. **Mantieni sessione attiva** - Usa `screen` o `tmux`:
   ```bash
   screen -S vision
   ./avvia_web_interface_con_vision.sh
   # Ctrl+A poi D per detach
   # Riconnetti: screen -r vision
   ```

2. **Porta diversa** - Se 8080 occupata:
   ```bash
   export WEB_PORT=8081
   ./avvia_web_interface_con_vision.sh
   ```

3. **Debug** - Avvia componenti separatamente per debugging

4. **Modelli YOLO** - Cambia modello:
   ```bash
   # In vision_yolo_detector.py, modifica:
   # yolo_model = 'yolov8n.pt'  # nano (veloce)
   # yolo_model = 'yolov8s.pt'  # small
   # yolo_model = 'yolov8m.pt'  # medium
   ```

## ✅ Checklist

Prima di usare:

- [ ] File deployati su AI Accelerator
- [ ] Dipendenze installate (YOLOv8, cv_bridge)
- [ ] Camera Orbecc collegata via USB
- [ ] Robot raggiungibile (ping 192.168.10.194)
- [ ] Script eseguibili (chmod +x *.sh)
- [ ] Test superato (python3 test_vision_system.py)

## 🎉 Pronto!

Ora puoi usare la tua web interface esistente **CON** le funzionalità vision!

```bash
./avvia_web_interface_con_vision.sh
```

Poi apri browser su: `http://192.168.10.191:8080` 🚀




