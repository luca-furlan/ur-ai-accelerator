# 🚀 ISTRUZIONI AVVIO WEB INTERFACE

## ✅ METODO RAPIDO

### 1. Connettiti all'AI Accelerator

```bash
ssh lab@192.168.10.191
```

### 2. Vai nella directory

```bash
cd ~/MekoAiAccelerator
```

### 3. Avvia Web Interface

**Opzione A - Script Python:**
```bash
python3 start_web_interface.py
```

**Opzione B - Diretto:**
```bash
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

**Opzione C - In Background:**
```bash
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &
```

### 4. Apri Browser

```
http://192.168.10.191:8081
```

---

## 📋 COSA VEDRAI

Nella web interface vedrai:

1. **ROS2 Monitor** (in alto)
   - Stato ROS2 bridge
   - Publish loop
   - Topic disponibili

2. **Robot Status** (sotto ROS2 Monitor) - **NUOVO!**
   - Robot Mode (RUNNING/IDLE/POWER_OFF)
   - Safety Mode (NORMAL/PROTECTIVE_STOP)
   - Program State (PLAYING/STOPPED)
   - Remote Control (true/false)
   - **Joint Positions** (aggiornati ogni 2 secondi)
   - **TCP Pose** (aggiornati ogni 2 secondi)

3. **Joystick e Controlli** (sotto)
   - Joystick virtuale per controllo robot
   - Controllo joint per joint
   - Movimento cartesiano

---

## 🔧 TROUBLESHOOTING

### Web Interface non si avvia

**Errore:** `ModuleNotFoundError` o `ImportError`

**Soluzione:**
```bash
# Installa dipendenze
pip3 install --user flask flask-cors
```

### Porta già in uso

**Errore:** `Address already in use`

**Soluzione:**
```bash
# Trova processo
ps aux | grep web_interface

# Ferma processo
pkill -f web_interface

# O usa porta diversa
export WEB_PORT=8082
python3 -m remote_ur_control.web_interface
```

### Robot Status mostra errori

**Problema:** Dashboard o RTDE non raggiungibili

**Soluzione:**
1. Verifica robot acceso: `ping 192.168.10.194`
2. Installa ur_rtde: `pip3 install --user ur-rtde`
3. Verifica porte robot aperte

---

## ✅ VERIFICA

Dopo aver avviato, verifica:

```bash
# Dalla tua macchina
curl http://192.168.10.191:8081/
# Dovresti vedere HTML

curl http://192.168.10.191:8081/api/status
# Dovresti vedere JSON con stato ROS2

curl http://192.168.10.191:8081/api/robot_status
# Dovresti vedere JSON con stato robot (joints, TCP, etc.)
```

---

## 🎯 COMANDI RAPIDI

**Avvia:**
```bash
ssh lab@192.168.10.191 "cd ~/MekoAiAccelerator && export UR_ROBOT_IP=192.168.10.194 && export WEB_PORT=8081 && python3 -m remote_ur_control.web_interface"
```

**Ferma:**
```bash
ssh lab@192.168.10.191 "pkill -f web_interface"
```

**Verifica log:**
```bash
ssh lab@192.168.10.191 "tail -f /tmp/web_interface.log"
```

---

**Ultimo aggiornamento:** $(date)










