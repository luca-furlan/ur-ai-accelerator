# 🎮 Controllo Fluido Robot UR - Guida Completa

## ✅ Il joystick simulato è già implementato e funzionante!

La web interface include:
- **2 Joystick virtuali** (XY e Z/Rotation)
- **Controllo fluido a 125Hz** tramite ROS2 bridge
- **Exponential smoothing** per movimento senza scatti
- **Fallback socket** se ROS2 non disponibile

## 🚀 Avvio in 3 passi

### 1. Connettiti all'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

### 2. Avvia web interface

```bash
cd ~/MekoAiAccelerator

# Source ROS2 (se disponibile)
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Configura
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Avvia
python3 -m remote_ur_control.web_interface
```

### 3. Apri browser

```
http://192.168.10.191:8080
```

## 🎮 Come usare il joystick

1. **Joystick XY** (sinistra):
   - Controlla joint 0 (base) e joint 1 (spalla)
   - Muovi mouse/touch per controllare direzione
   - Movimento fluido in tempo reale

2. **Joystick Z/Rotation** (destra):
   - Controlla movimento Z e rotazione
   - Stesso funzionamento del joystick XY

3. **Pulsanti ±**:
   - Controllo manuale di ogni joint
   - Step configurabile

4. **Emergency Stop**:
   - Ferma immediatamente tutti i movimenti

## ⚡ Caratteristiche controllo fluido

- **125Hz publishing rate** - movimento ultra fluido
- **Exponential smoothing** - nessun movimento a scatti
- **Real-time feedback** - vedi stato robot in tempo reale
- **Dual mode**: Joint control o Cartesian control

## 🔧 Troubleshooting

### Robot non si muove?

1. Verifica che robot sia in RUNNING:
   ```bash
   # Sul teach pendant: avvia programma con External Control
   ```

2. Verifica connessione:
   ```bash
   ping 192.168.10.194
   ```

3. Verifica porta 30002:
   ```bash
   timeout 2 bash -c "</dev/tcp/192.168.10.194/30002" && echo "OK" || echo "FAIL"
   ```

### Web interface non si avvia?

1. Installa Flask:
   ```bash
   pip3 install --user flask flask-cors
   ```

2. Verifica file:
   ```bash
   ls -la ~/MekoAiAccelerator/remote_ur_control/web_interface.py
   ```

### Porta 8080 occupata?

```bash
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

Poi apri: `http://192.168.10.191:8081`

## 📊 Monitoraggio

La web interface mostra:
- ✅ Stato connessione robot
- ✅ Stato ROS2 bridge (se disponibile)
- ✅ Ultimo comando inviato
- ✅ Velocità corrente dei joint
- ✅ Frequenza pubblicazione (125Hz)

## 🎯 Prossimi passi

Una volta che il controllo base funziona:
1. ✅ **Già fatto**: Joystick simulato funzionante
2. 🔄 Integra camera Orbbec per controllo visivo
3. 🔄 Aggiungi AI/ML per riconoscimento oggetti
4. 🔄 Implementa teleoperazione avanzata

## 📝 Note tecniche

- Il controllo usa **speedj** per movimento fluido
- Se ROS2 disponibile: pubblica a 125Hz su `/forward_velocity_controller/commands`
- Se ROS2 non disponibile: usa socket diretto (meno fluido ma funziona)
- Exponential smoothing evita movimenti a scatti

