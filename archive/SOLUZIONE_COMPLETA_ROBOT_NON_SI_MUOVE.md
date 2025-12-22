# 🔧 SOLUZIONE COMPLETA - Robot Non Si Muove

## ❌ PROBLEMA PRINCIPALE

Il robot non si muove perché:
1. **Programma NON in PLAYING** (è STOPPED) - **QUESTO È IL PROBLEMA PRINCIPALE**
2. Driver UR ROS2 non in esecuzione (opzionale, se vuoi usare ROS2)

## ✅ SOLUZIONE RAPIDA (SOCKET DIRETTO)

La web interface è già configurata per usare socket diretto. **Basta mettere il programma in PLAYING!**

### PASSO 1: Sul Teach Pendant
1. Avvia il programma (es. "remote_control.urp")
2. **Metti in PLAYING** (premi PLAY)
3. Verifica che "Remote Control" sia attivo

### PASSO 2: Riavvia Web Interface
```bash
ssh lab@192.168.10.191
pkill -f web_interface
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

### PASSO 3: Controlla Robot
- Apri: http://192.168.10.191:8081
- Verifica "Program State: PLAYING"
- Usa joystick - **DOVREBBE FUNZIONARE!**

---

## 🔧 SOLUZIONE AVANZATA (ROS2 Driver - Documentazione Ufficiale)

Se vuoi usare ROS2 Driver (più complesso ma più potente):

### SETUP ROBOT (Teach Pendant)

1. **Installa External Control URCap** sul robot
   - Vai su: **Installation** → **URCaps**
   - Installa "External Control"

2. **Crea Programma con External Control**
   - Crea nuovo programma
   - Aggiungi nodo **External Control**
   - Configura:
     - **IP Host**: `192.168.10.191` (IP AI Accelerator)
     - **Porta**: `50002`
   - Salva come `ros_control.urp`

3. **Avvia Programma**
   - Metti in **PLAYING**
   - Verifica connessione

### SETUP AI ACCELERATOR (3 Terminali)

#### TERMINALE 1 - Driver UR ROS2
```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Aspetta che vedi:**
```
[INFO] [ur_robot_driver]: Robot connected
[INFO] [ur_robot_driver]: Controllers started
```

#### TERMINALE 2 - Test Movimento (Opzionale)
```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

Il robot dovrebbe muoversi dopo pochi secondi.

#### TERMINALE 3 - Web Interface (con ROS2)
```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

### Verifica Controller ROS2

```bash
# Installa tool (se non presente)
sudo apt install ros-humble-ros2controlcli

# Verifica controller
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

Dovresti vedere:
```
scaled_joint_trajectory_controller [active]
forward_velocity_controller [active]
...
```

---

## 📋 CHECKLIST

### Per Socket Diretto (SOLUZIONE SEMPLICE)
- [ ] Programma in PLAYING sul teach pendant
- [ ] Web interface avviata
- [ ] Robot raggiungibile (ping 192.168.10.194)

### Per ROS2 Driver (SOLUZIONE AVANZATA)
- [ ] External Control URCap installato sul robot
- [ ] Programma con External Control configurato
- [ ] IP Host: 192.168.10.191, Porta: 50002
- [ ] Programma in PLAYING
- [ ] Driver UR ROS2 in esecuzione (Terminale 1)
- [ ] Controller ROS2 attivi (`ros2 control list_controllers`)

---

## 🐛 DEBUG

### Verifica Stato Robot
```bash
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.10.194", 29999))
sock.recv(1024)
sock.sendall(b"programState\n")
print(sock.recv(1024).decode())
sock.close()
PYTHON
```

Dovresti vedere: `PLAYING`

### Verifica Connessione ROS2
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep forward_velocity
ros2 topic echo /forward_velocity_controller/commands
```

Se vedi messaggi quando muovi joystick → ROS2 funziona!

### Test Socket Diretto
```bash
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.10.194", 30002))
script = "speedj([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\n"
sock.sendall(script.encode('utf-8'))
sock.close()
print("Comando inviato - robot dovrebbe muoversi leggermente")
PYTHON
```

Se robot si muove → Socket funziona!

---

## 💡 RACCOMANDAZIONE

**USA SOCKET DIRETTO** (più semplice):
- ✅ Funziona sempre se programma è PLAYING
- ✅ Nessuna configurazione complessa
- ✅ Controllo fluido a 125Hz
- ✅ Già configurato nella web interface

**ROS2 Driver** solo se:
- Hai bisogno di integrazione MoveIt
- Vuoi usare altri nodi ROS2
- Hai già configurato External Control URCap

---

## 📞 SUPPORTO

Se ancora non funziona:
1. Verifica log: `tail -f /tmp/web_interface.log`
2. Verifica connessione: `ping 192.168.10.194`
3. Verifica stato robot: vedi sezione DEBUG sopra










