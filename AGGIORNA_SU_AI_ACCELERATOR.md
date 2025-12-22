# 🔄 Aggiornamento su AI Accelerator

## ✅ Modifiche Caricate

Tutte le modifiche sono state committate e pushato sul repository remoto.

**Ultimi commit:**
- ✅ Sistema vision YOLO completo (detector, subscriber, comando avvicinati)
- ✅ Web interface gestisce tutti i processi (camera, YOLO, MoveIt)
- ✅ FERMA_TUTTO.sh aggiornato

---

## 📥 Come Aggiornare sul Server AI Accelerator

### 1. Connettiti al server:
```bash
ssh lab@<IP_AI_ACCELERATOR>
```

### 2. Vai nella directory del progetto:
```bash
cd ~/MekoAiAccelerator
```

### 3. Ferma tutto (se necessario):
```bash
bash FERMA_TUTTO.sh
```

### 4. Aggiorna il codice:
```bash
git pull origin feature/remote-ur-control
```

**Oppure semplicemente:**
```bash
git pull
```

### 5. Verifica che i nuovi file siano presenti:
```bash
ls -la vision_yolo_detector.py
ls -la avvia_vision_system.sh
ls -la SISTEMA_VISION_COMPLETO.md
```

### 6. Rendi eseguibile lo script (se necessario):
```bash
chmod +x vision_yolo_detector.py
chmod +x avvia_vision_system.sh
chmod +x FERMA_TUTTO.sh
```

### 7. Avvia web interface:
```bash
bash avvia_web_interface.sh
```

---

## 🎯 Cosa è Cambiato

### Nuovi File:
- ✅ `vision_yolo_detector.py` - Nodo ROS2 per YOLO detection
- ✅ `avvia_vision_system.sh` - Script per avviare vision system
- ✅ `SISTEMA_VISION_COMPLETO.md` - Documentazione completa

### File Modificati:
- ✅ `remote_ur_control/web_interface.py` - Gestione completa processi
- ✅ `FERMA_TUTTO.sh` - Ferma anche camera, YOLO, MoveIt

### Nuove Funzionalità:
1. **Gestione Processi dalla Web Interface:**
   - Start/Stop Camera Orbbec
   - Start/Stop YOLO Detector
   - Start/Stop MoveIt
   - Status in tempo reale

2. **Sistema Vision Completo:**
   - YOLO detection con coordinate 3D
   - Comando "Avvicinati" al pezzo (20cm)
   - Integrazione MoveIt per movimento automatico

---

## 🚀 Test Rapido

Dopo `git pull` e avvio web interface:

1. Apri browser: `http://<IP_AI_ACCELERATOR>:8080`
2. Vai a sezione "Vision System & MoveIt"
3. Clicca "Start Camera" → dovrebbe avviare camera Orbbec
4. Clicca "Start YOLO Detector" → dovrebbe avviare YOLO
5. Clicca "Start MoveIt" → dovrebbe avviare MoveIt

Se vedi errori, controlla i log:
```bash
tail -f /tmp/orbbec_camera.log
tail -f /tmp/yolo_detector.log
tail -f /tmp/moveit.log
```

---

## ⚠️ Note Importanti

1. **Prima volta:** Potrebbe essere necessario installare YOLOv8:
   ```bash
   pip3 install ultralytics
   ```

2. **ROS2 Environment:** Assicurati che ROS2 sia configurato:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **Permessi:** Se ci sono errori di permessi:
   ```bash
   chmod +x vision_yolo_detector.py
   ```

---

**Ultimo aggiornamento:** 2025-12-22
