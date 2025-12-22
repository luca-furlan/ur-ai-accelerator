# 🚀 Deploy e Utilizzo Sistema Vision + Robot

## 📍 Architettura

**TUTTO sull'AI Accelerator** (Linux remoto):
- ✅ ROS2 + MoveIt2
- ✅ Camera Orbecc + Vision YOLO
- ✅ Robot Control
- ✅ Web Interface Flask

**Windows** (tua macchina):
- ✅ Solo browser web → `http://AI_ACCELERATOR_IP:5000`

---

## 🎯 Setup Completo (3 passi)

### **PASSO 1: Deploy da Windows**

#### Opzione A: Con Git Bash / WSL (raccomandato)

```bash
# Configura IP e user
export AI_ACCELERATOR_IP=192.168.1.100  # ← MODIFICA CON IP REALE
export AI_ACCELERATOR_USER=user         # ← MODIFICA CON TUO USERNAME

# Deploy tutto
bash deploy_vision_completo.sh
```

#### Opzione B: Manuale con WinSCP/FileZilla

1. Connetti a AI Accelerator via SFTP
2. Carica questi file in `~/MekoAiAccelerator/`:
   ```
   vision_yolo_detector.py
   moveit_vision_controller.py
   vision_robot_coordinator.py
   launch_vision_robot_system.py
   avvia_vision_robot_system.sh
   ferma_vision_robot_system.sh
   integra_vision_web_interface.py
   test_vision_system.py
   remote_ur_control/
   ```

---

### **PASSO 2: Avvia su AI Accelerator**

```bash
# SSH su AI Accelerator
ssh user@192.168.1.100

# Vai nella directory
cd ~/MekoAiAccelerator

# Rendi eseguibili gli script (prima volta)
chmod +x *.sh *.py

# Installa dipendenze (prima volta)
pip install ultralytics opencv-python numpy flask requests
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs

# Avvia TUTTO (vision + robot + web interface)
bash avvia_sistema_completo.sh
```

**Output atteso:**
```
==========================================
AVVIO SISTEMA COMPLETO
==========================================

[1/2] Avvio vision system...
[2/2] Setup web interface...

✅ Avvio web interface...

==========================================
Sistema ONLINE!
==========================================

Accedi da browser:
  http://192.168.1.100:5000

Vision system PID: 12345
Log: /tmp/vision_system.log

Per fermare: Ctrl+C o ./ferma_sistema_completo.sh
==========================================
```

---

### **PASSO 3: Usa da Windows**

Apri browser e vai a:

```
http://192.168.1.100:5000
```

(Sostituisci `192.168.1.100` con IP reale dell'AI Accelerator)

**Vedrai la web interface con:**
- 📊 Status sistema (vision + robot)
- 👁️ Detections real-time
- 🎮 Controlli pick/stop
- 📹 Video feed annotato

---

## 🎮 Utilizzo

### Via Browser (interfaccia grafica)

1. Apri `http://AI_ACCELERATOR_IP:5000`
2. Visualizza detections in tempo reale
3. Seleziona classe oggetto (es: "bottle")
4. Click "Pick Target" per far muovere il robot

### Via API REST (da codice/script)

```bash
# Status sistema
curl http://192.168.1.100:5000/api/vision/status

# Detections correnti
curl http://192.168.1.100:api/vision/detections

# Seleziona classe
curl -X POST http://192.168.1.100:5000/api/vision/select_class \
  -H "Content-Type: application/json" \
  -d '{"class": "bottle"}'

# Pick oggetto
curl -X POST http://192.168.1.100:5000/api/vision/pick_target

# Stop robot
curl -X POST http://192.168.1.100:5000/api/vision/stop
```

---

## 🛑 Stop Sistema

Su AI Accelerator (via SSH):

```bash
# Stop completo
bash ferma_sistema_completo.sh

# Oppure Ctrl+C nel terminale dove gira
```

---

## 📊 Monitoraggio

### Log sistema

```bash
# SSH su AI Accelerator
ssh user@192.168.1.100

# Log vision system
tail -f /tmp/vision_system.log

# Log singoli componenti
tail -f /tmp/vision_detector.log
tail -f /tmp/moveit_controller.log
tail -f /tmp/coordinator.log
```

### ROS2 Topics

```bash
# SSH su AI Accelerator
ssh user@192.168.1.100

# Monitor detections
ros2 topic echo /vision/detections_3d

# Monitor status
ros2 topic echo /vision/system_status

# Visualizza immagine annotata
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

---

## 🔧 Troubleshooting

### Sistema non si avvia

```bash
# SSH su AI Accelerator
ssh user@192.168.1.100
cd ~/MekoAiAccelerator

# Test componenti
python3 test_vision_system.py

# Verifica ROS2
source /opt/ros/humble/setup.bash
ros2 topic list

# Verifica camera
ros2 topic list | grep camera
```

### Web interface non raggiungibile da Windows

```bash
# Da Windows: verifica connessione
ping 192.168.1.100

# Su AI Accelerator: verifica porta
netstat -tlnp | grep 5000

# Su AI Accelerator: verifica firewall
sudo ufw status
sudo ufw allow 5000/tcp  # se necessario
```

### Camera non funziona

```bash
# Su AI Accelerator
lsusb | grep Orbbec

# Riavvia driver camera
pkill -f orbbec_camera
ros2 launch orbbec_camera gemini_330_series.launch.py
```

### Robot non risponde

```bash
# Verifica connessione robot
ping 192.168.10.194

# Check status robot
python3 remote_ur_control/check_robot_status.py
```

---

## 📝 Configurazione

### Variabili ambiente (su AI Accelerator)

Modifica in `avvia_sistema_completo.sh`:

```bash
export ROBOT_IP=192.168.10.194      # IP robot UR
export AUTO_MODE=false               # false=manuale, true=automatico
export YOLO_MODEL=yolov8n.pt        # Modello YOLO
export CONFIDENCE=0.5                # Threshold confidence
```

### Modelli YOLO disponibili

- `yolov8n.pt` - Nano (velocissimo) ⭐ **RACCOMANDATO**
- `yolov8s.pt` - Small (veloce)
- `yolov8m.pt` - Medium
- `yolov8l.pt` - Large (accurato ma lento)

---

## 🎯 Workflow Tipico

### 1. Avvio giornaliero

```bash
# Su AI Accelerator (via SSH)
cd ~/MekoAiAccelerator
bash avvia_sistema_completo.sh

# Su Windows
# Apri browser: http://192.168.1.100:5000
```

### 2. Pick oggetto

1. Metti oggetto davanti camera
2. Attendi detection (visibile su web interface)
3. Click su "Select Class" e scegli classe (es: "bottle")
4. Click "Pick Target"
5. Robot si muove verso oggetto

### 3. Stop giornaliero

```bash
# Su AI Accelerator
bash ferma_sistema_completo.sh
```

---

## 📚 Documentazione Completa

- **README_VISION_ROBOT.md** - Dettagli tecnici architettura
- **QUICK_START_VISION.md** - Tutorial step-by-step
- **ARCHITETTURA_DEPLOY.md** - Schema architettura
- **VISION_SYSTEM_COMPLETE.md** - Reference completo

---

## ✅ Checklist Pre-Utilizzo

Su AI Accelerator:

- [ ] ROS2 Humble installato
- [ ] Camera Orbecc collegata via USB
- [ ] Driver orbbec_camera installato
- [ ] Python packages installati (ultralytics, opencv, flask)
- [ ] Robot UR raggiungibile (ping 192.168.10.194)
- [ ] File deployati in ~/MekoAiAccelerator
- [ ] Script resi eseguibili (chmod +x *.sh)
- [ ] Test sistema passato (python3 test_vision_system.py)

Da Windows:

- [ ] AI Accelerator raggiungibile (ping IP)
- [ ] Porta 5000 accessibile
- [ ] Browser moderno (Chrome/Firefox)

---

## 🎉 Quick Start Ultra-Rapido

```bash
# 1. Deploy da Windows (Git Bash/WSL)
export AI_ACCELERATOR_IP=192.168.1.100
bash deploy_vision_completo.sh

# 2. SSH e avvia
ssh user@192.168.1.100
cd ~/MekoAiAccelerator
bash avvia_sistema_completo.sh

# 3. Su Windows - apri browser
# http://192.168.1.100:5000
```

**Fatto!** Sistema pronto all'uso 🚀

---

## 💡 Tips

1. **Bookmark URL** - Salva `http://AI_ACCELERATOR_IP:5000` nei preferiti
2. **Screen session** - Usa `screen` o `tmux` per mantenere sistema attivo dopo logout SSH
3. **Auto-start** - Aggiungi a systemd per avvio automatico al boot
4. **Backup config** - Salva configurazioni personalizzate
5. **Multiple clients** - Più browser possono connettersi simultaneamente

---

## 📞 Supporto

**Test sistema:**
```bash
python3 test_vision_system.py
```

**Logs:**
```bash
tail -f /tmp/vision_system.log
```

**Status ROS2:**
```bash
ros2 node list
ros2 topic list
```

---

**Sistema pronto per produzione!** 🎯




