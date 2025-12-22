# 🔧 Robot Non Si Muove - Diagnostica e Soluzione

## Problema
Il joystick funziona ma il robot non si muove. Le velocità target rimangono a 0.

## Diagnostica Completa

Esegui questo script per vedere cosa non funziona:

```bash
cd ~/MekoAiAccelerator
bash diagnostica_completa.sh
```

## Cause Comuni

### 1. Driver UR ROS2 Non In Esecuzione

**Sintomo**: Topic `/forward_velocity_controller/commands` non esiste

**Soluzione**:
```bash
# Terminale 1: Avvia driver UR
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false
```

**Sul Teach Pendant**:
1. Avvia programma con **External Control**
2. IP Host: `192.168.10.191`
3. Porta: `50002`
4. Premi **PLAY**

### 2. Robot Non In Modalità Corretta

**Sintomo**: Robot Mode = POWER_OFF o Program State = STOPPED

**Soluzione**:
```bash
# Verifica stato
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.10.194", 29999))
sock.recv(1024)
sock.sendall(b"robotmode\n")
print(sock.recv(1024).decode())
sock.sendall(b"programState\n")
print(sock.recv(1024).decode())
sock.close()
PYTHON
```

Se POWER_OFF:
```bash
# Accendi robot
python3 ~/MekoAiAccelerator/remote_ur_control/power_on_robot.py
```

### 3. Topic ROS2 Non Riceve Messaggi

**Verifica**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Vedi se il topic esiste
ros2 topic list | grep forward_velocity

# Monitora messaggi in tempo reale
ros2 topic echo /forward_velocity_controller/commands
```

**Se non vedi messaggi quando muovi il joystick**, il problema è nella pubblicazione.

### 4. Web Interface Non Pubblica

**Verifica log**:
```bash
tail -f /tmp/web_interface.log
```

Dovresti vedere:
```
📤 Published X messages (current speeds: [...])
```

**Se non vedi pubblicazioni**, c'è un problema con il bridge ROS2.

## Test Rapido

### Test 1: Comando Diretto Socket

```bash
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.10.194", 30002))
script = "speedj([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\n"
sock.sendall(script.encode('utf-8'))
sock.close()
print("Comando inviato - il robot dovrebbe muoversi leggermente")
PYTHON
```

**Se il robot si muove**: Il problema è nel bridge ROS2
**Se il robot NON si muove**: Il problema è nel robot/configurazione

### Test 2: Pubblicazione ROS2 Diretta

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Pubblica comando direttamente
ros2 topic pub --once /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Se il robot si muove**: Il problema è nella web interface
**Se il robot NON si muove**: Il problema è nel driver UR o robot

## Soluzione Completa

### Setup Corretto (3 Terminali)

**Terminale 1 - Driver UR**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194
```

**Terminale 2 - Web Interface**:
```bash
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

**Terminale 3 - Monitor**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /forward_velocity_controller/commands
```

### Verifica Funzionamento

1. Apri browser: `http://192.168.10.191:8081`
2. Muovi il joystick
3. Nel Terminale 3 dovresti vedere messaggi con velocità diverse da 0
4. Il robot dovrebbe muoversi

## Checklist Finale

- [ ] Robot in RUNNING (non POWER_OFF)
- [ ] Programma sul teach pendant in PLAYING
- [ ] Driver UR ROS2 in esecuzione
- [ ] Topic `/forward_velocity_controller/commands` esiste
- [ ] Web interface pubblica messaggi (vedi log)
- [ ] Terminale monitor mostra messaggi quando muovi joystick
- [ ] Robot si muove quando pubblichi direttamente su topic

