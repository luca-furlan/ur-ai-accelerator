# ✅ Sistema Vision Completo - YOLO + Riconoscimento + Avvicinamento

## 🎯 Funzionalità Implementate

### 1. **Nodo YOLO Detector** (`vision_yolo_detector.py`)
- ✅ Sottoscrive a `/camera/color/image_raw` e `/camera/depth/image_raw`
- ✅ Esegue detection YOLOv8 su ogni frame
- ✅ Calcola coordinate 3D usando depth camera
- ✅ Pubblica detections su `/vision/detections_3d` (JSON)
- ✅ Pubblica immagini annotate su `/vision/annotated_image`

### 2. **Subscriber Detections** (in `web_interface.py`)
- ✅ Thread separato che legge `/vision/detections_3d`
- ✅ Mantiene ultime detections in memoria thread-safe
- ✅ Endpoint `/api/vision/detections` restituisce detections formattate

### 3. **Comando "Avvicinati"** (`/api/vision/approach_object`)
- ✅ Legge ultime detections
- ✅ Seleziona pezzo più vicino (o quello selezionato dall'utente)
- ✅ Calcola posizione target a 20cm dal pezzo
- ✅ Calcola orientamento per guardare verso il pezzo
- ✅ Usa MoveIt per pianificare ed eseguire movimento
- ✅ Trasforma coordinate camera → base robot

### 4. **Interfaccia Web**
- ✅ Visualizzazione detections nella sezione "Object Detections"
- ✅ Click su detection per selezionarla
- ✅ Pulsante "Avvicinati al Pezzo (20cm)"
- ✅ Status box per feedback movimento

---

## 🔄 Flusso Completo

### Scenario: Riconoscere pezzo e avvicinarsi

1. **Avvia Camera:**
   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py
   ```
   Oppure usa pulsante "Start Camera" nella web interface

2. **Avvia YOLO Detector:**
   ```bash
   bash avvia_vision_system.sh
   ```
   Oppure clicca "Start Vision System" nella web interface

3. **YOLO Riconosce Pezzi:**
   - YOLO detector legge frame dalla camera
   - Esegue inference YOLOv8
   - Calcola coordinate 3D usando depth
   - Pubblica su `/vision/detections_3d`

4. **Web Interface Mostra Detections:**
   - Subscriber legge detections
   - Mostra nella sezione "Object Detections"
   - Ogni detection mostra: classe, confidence, posizione 3D

5. **Utente Clicca "Avvicinati":**
   - Seleziona pezzo più vicino (o quello selezionato)
   - Calcola posizione target: `pezzo - (direzione * 0.20m)`
   - Calcola orientamento per guardare verso il pezzo
   - Usa MoveIt per muovere robot

6. **Robot Si Muove:**
   - MoveIt pianifica traiettoria
   - Esegue movimento verso target
   - Robot si posiziona a 20cm dal pezzo

---

## 📋 Componenti

### `vision_yolo_detector.py`
Nodo ROS2 standalone che:
- Si sottoscrive a camera RGB + Depth
- Esegue YOLOv8 detection
- Calcola coordinate 3D
- Pubblica detections

**Avvio:**
```bash
source /opt/ros/humble/setup.bash
python3 vision_yolo_detector.py
```

**Oppure:**
```bash
bash avvia_vision_system.sh
```

### `web_interface.py` - Endpoints

#### `/api/vision/detections` (GET)
Restituisce ultime detections formattate.

**Response:**
```json
{
  "status": "ok",
  "detections": [
    {
      "class_name": "bottle",
      "confidence": 0.85,
      "position": {"x": 0.3, "y": 0.1, "z": 0.5},
      "bbox_2d": {...},
      "center_2d": {...}
    }
  ]
}
```

#### `/api/vision/approach_object` (POST)
Avvicina robot a un pezzo.

**Request:**
```json
{
  "distance": 0.20,  // 20cm
  "selected_index": 0  // opzionale: indice detection selezionata
}
```

**Response:**
```json
{
  "status": "ok",
  "message": "Robot avvicinato a bottle (distanza: 20cm)",
  "data": {
    "object_class": "bottle",
    "object_position": {"x": 0.3, "y": 0.1, "z": 0.5},
    "target_position": {"x": 0.28, "y": 0.09, "z": 0.48},
    "distance": 0.20,
    "executed": true
  }
}
```

---

## 🚀 Come Usare

### Setup Completo:

1. **Avvia Camera:**
   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py
   ```

2. **Avvia YOLO Detector:**
   ```bash
   cd ~/MekoAiAccelerator
   bash avvia_vision_system.sh
   ```

3. **Avvia MoveIt** (se non già attivo):
   ```bash
   ros2 launch ur_moveit_config moveit.launch.py
   ```

4. **Avvia Web Interface:**
   ```bash
   bash avvia_web_interface.sh
   ```

5. **Nella Web Interface:**
   - Vai a sezione "Vision System & MoveIt"
   - Clicca "Start Camera" (se non già attiva)
   - Clicca "Start Vision System"
   - Attendi che appaiano detections nella sezione "Object Detections"
   - (Opzionale) Clicca su una detection per selezionarla
   - Clicca "Avvicinati al Pezzo (20cm)"
   - Il robot si muoverà automaticamente verso il pezzo

---

## 🔧 Dettagli Implementazione

### Calcolo Posizione Target

```python
# Posizione pezzo (in frame camera)
object_x, object_y, object_z = posizione_3d_pezzo

# Distanza dal robot
distance_to_object = sqrt(object_x² + object_y² + object_z²)

# Vettore direzione normalizzato
direction_x = object_x / distance_to_object
direction_y = object_y / distance_to_object
direction_z = object_z / distance_to_object

# Posizione target: pezzo - (direzione * target_distance)
target_x = object_x - direction_x * 0.20  # 20cm
target_y = object_y - direction_y * 0.20
target_z = object_z - direction_z * 0.20
```

### Calcolo Orientamento

```python
# Vettore da target a pezzo
look_x = direction_x
look_y = direction_y
look_z = direction_z

# Yaw: rotazione attorno Z (base)
yaw = atan2(look_y, look_x)

# Pitch: rotazione attorno Y (inclinazione)
pitch = -asin(look_z)

# Roll: 0 (no rotazione attorno X)
roll = 0.0
```

### Trasformazione Coordinate

**⚠️ NOTA IMPORTANTE:**
Attualmente il sistema assume che le coordinate della camera siano già nel frame `base_link`. 

**Per produzione:**
- Implementare trasformazione usando `tf2_ros.TransformListener`
- Trasformare da `camera_color_optical_frame` → `base_link`
- Usare transform lookup per coordinate corrette

**Esempio futuro:**
```python
from tf2_ros import TransformListener, Buffer
listener = TransformListener(buffer, node)
transform = buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
# Applica transform a coordinate pezzo
```

---

## ⚙️ Requisiti

### Software:
- ROS2 Humble
- YOLOv8: `pip install ultralytics`
- cv_bridge: `sudo apt install ros-humble-cv-bridge`
- MoveIt2: `sudo apt install ros-humble-moveit`
- Camera Orbbec driver ROS2

### Hardware:
- Camera Orbbec con RGB + Depth
- Robot UR5e configurato con MoveIt

---

## 🔍 Debug

### Verifica YOLO Detector:
```bash
# Verifica processo
pgrep -f vision_yolo_detector

# Log
tail -f /tmp/yolo_detector.log

# Verifica topic
ros2 topic echo /vision/detections_3d
```

### Verifica Detections:
```bash
# Nella web interface, apri console browser (F12)
# Dovresti vedere detections aggiornate ogni 2 secondi
```

### Verifica Movimento:
- Controlla log web interface per messaggi `[APPROACH]`
- Verifica MoveIt action server: `ros2 action list | grep move_action`
- Controlla planning: `ros2 topic echo /move_action/_action/status`

---

## ⚠️ Note Importanti

1. **Trasformazione Coordinate:**
   - Attualmente assume coordinate camera = base robot
   - In produzione, implementare tf2 transform

2. **Selezione Pezzo:**
   - Se nessun pezzo selezionato, usa il più vicino
   - Se pezzo selezionato, usa quello

3. **Distanza Target:**
   - Default: 20cm (0.20m)
   - Configurabile nel request

4. **Orientamento:**
   - Tool0 (end effector) punta verso il pezzo
   - Calcolato automaticamente dalla direzione

5. **Sicurezza:**
   - MoveIt gestisce collisioni e limiti
   - Verifica sempre che il target sia raggiungibile
   - Controlla che il pezzo sia a distanza ragionevole (>10cm)

---

**Ultimo aggiornamento:** 2025-12-22
