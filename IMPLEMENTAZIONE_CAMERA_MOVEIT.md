# ✅ Implementazione Camera Stream e MoveIt

## 📹 Camera Stream - IMPLEMENTATO

### Cosa è stato fatto:
1. **Subscriber ROS2 per camera stream**
   - Thread separato che si sottoscrive al topic `/camera/color/image_raw`
   - Converte frame ROS2 Image → OpenCV → JPEG
   - Mantiene ultimo frame in memoria thread-safe
   - Inizializzazione automatica all'avvio web interface

2. **Endpoint `/api/vision/camera_stream`**
   - Restituisce ultimo frame JPEG disponibile
   - Se nessun frame disponibile, restituisce placeholder
   - Aggiornamento automatico frontend ogni 100ms (10 FPS)

### Come funziona:
- All'avvio della web interface, viene creato un subscriber ROS2 in background
- Il subscriber riceve frame dalla camera e li converte in JPEG
- L'endpoint Flask legge l'ultimo frame e lo restituisce al browser
- Il frontend aggiorna l'immagine automaticamente

### Requisiti:
- Camera Orbbec attiva e pubblicante su `/camera/color/image_raw`
- `cv_bridge` installato: `sudo apt install ros-humble-cv-bridge`
- ROS2 environment configurato

---

## 🤖 MoveIt Planning & Execution - IMPLEMENTATO

### Cosa è stato fatto:
1. **Pianificazione movimento**
   - Converte target pose (x, y, z, roll, pitch, yaw) in quaternion
   - Crea goal per MoveIt action `move_action`
   - Imposta planning group `ur_manipulator`
   - Configura constraints per posizione e orientamento

2. **Esecuzione movimento**
   - Se `execute=true`, esegue il movimento pianificato
   - Se `execute=false`, solo planning (non esegue)
   - Gestione errori e timeout
   - Messaggi di errore dettagliati

### Come funziona:
- L'endpoint `/api/vision/plan_move` riceve target pose
- Crea un action client per MoveIt
- Invia goal al server MoveIt
- Attende risultato (planning o execution)
- Restituisce status e dettagli

### Requisiti:
- MoveIt configurato e running: `ros2 launch ur_moveit_config moveit.launch.py`
- MoveIt action server attivo su `move_action`
- Planning group `ur_manipulator` configurato
- MoveIt Python packages: `sudo apt install ros-humble-moveit`

### Esempio uso:
```json
POST /api/vision/plan_move
{
  "target_pose": {
    "x": 0.3,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "execute": true
}
```

---

## 🔍 Vision System (YOLO) - PARZIALE

### Cosa è stato fatto:
- Endpoint `/api/vision/start` verifica che camera sia attiva
- Messaggio informativo più chiaro

### Cosa manca:
- Integrazione nodo YOLO per detections
- Parsing detections da topic ROS2
- Visualizzazione detections nell'interfaccia

### Prossimi passi:
1. Verificare se esiste nodo YOLO che pubblica su `/vision/detections_3d`
2. Se esiste, aggiungere subscriber per leggere detections
3. Se non esiste, creare nodo YOLO separato che:
   - Legge frame da `/camera/color/image_raw`
   - Esegue YOLO inference
   - Pubblica detections su `/vision/detections_3d`

---

## 🚀 Come Testare

### Camera Stream:
1. Avvia camera Orbbec:
   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py
   ```

2. Avvia web interface:
   ```bash
   bash avvia_web_interface.sh
   ```

3. Apri browser: `http://192.168.10.191:8080`

4. Vai a sezione "Vision System & MoveIt"

5. Clicca "Start Camera" (se non già attiva)

6. Lo stream video dovrebbe apparire automaticamente

### MoveIt:
1. Avvia MoveIt:
   ```bash
   ros2 launch ur_moveit_config moveit.launch.py
   ```

2. Nella web interface, vai a "MoveIt2 Motion Planning"

3. Inserisci target pose (es: x=0.3, y=0.0, z=0.3)

4. Clicca "Plan & Execute"

5. Il robot dovrebbe muoversi verso la posizione target

---

## ⚠️ Note Importanti

1. **Camera Stream:**
   - Se non vedi lo stream, verifica che la camera sia attiva: `ros2 topic list | grep camera`
   - Se vedi placeholder, il subscriber potrebbe non aver ricevuto frame ancora (attendi qualche secondo)

2. **MoveIt:**
   - MoveIt deve essere avviato PRIMA di usare la web interface
   - Se ottieni errori, verifica: `ros2 service list | grep moveit`
   - Il planning può richiedere alcuni secondi

3. **Performance:**
   - Camera stream: ~10 FPS (aggiornamento ogni 100ms)
   - MoveIt planning: ~5-10 secondi tipicamente
   - MoveIt execution: dipende dalla distanza e complessità del movimento

---

**Ultimo aggiornamento:** 2025-12-22
