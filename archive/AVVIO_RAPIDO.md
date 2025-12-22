# 🚀 Avvio Rapido - Controllo Fluido Robot UR

## Controllo immediato con joystick simulato

### Passo 1: Connettiti all'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

### Passo 2: Avvia controllo fluido

```bash
cd ~/MekoAiAccelerator
bash start_control_fluido.sh
```

**OPPURE manualmente:**

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

### Passo 3: Apri browser

Apri il browser e vai su:
```
http://192.168.10.191:8080
```

### Passo 4: Usa il joystick!

Nella pagina web vedrai:
- **Joystick XY** - controlla joint 0 e 1
- **Joystick Z/Rotation** - controlla movimento Z e rotazione
- **Pulsanti ±** - per ogni joint
- **Emergency Stop** - ferma tutto

## 🎮 Come funziona il joystick

1. **Clicca e tieni premuto** sul joystick
2. **Muovi il mouse** per controllare la direzione
3. **Rilascia** per fermare
4. Il robot si muove in **tempo reale** con controllo fluido a 125Hz

## ⚡ Controllo fluido garantito

- **125Hz publishing rate** - movimento ultra fluido
- **Exponential smoothing** - nessun movimento a scatti
- **Real-time feedback** - vedi lo stato del robot in tempo reale

## 🔧 Se qualcosa non funziona

### Robot non si muove?

1. Verifica che il robot sia acceso e in modalità RUNNING
2. Verifica che ci sia un programma attivo sul teach pendant
3. Controlla la connessione:
   ```bash
   ping 192.168.10.194
   ```

### Web interface non si avvia?

1. Installa Flask:
   ```bash
   pip3 install --user flask flask-cors
   ```

2. Verifica che i file siano presenti:
   ```bash
   ls -la ~/MekoAiAccelerator/remote_ur_control/web_interface.py
   ```

### Porta 8080 occupata?

Cambia porta:
```bash
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

Poi apri: `http://192.168.10.191:8081`

## 📊 Monitoraggio stato

La web interface mostra in tempo reale:
- ✅ Stato connessione robot
- ✅ Stato ROS2 bridge (se disponibile)
- ✅ Ultimo comando inviato
- ✅ Velocità corrente dei joint

## 🎯 Prossimi passi

Una volta che il controllo base funziona:
1. Integra camera Orbbec per controllo visivo
2. Aggiungi AI/ML per riconoscimento oggetti
3. Implementa teleoperazione avanzata

