# Istruzioni Deploy su AI Accelerator

## Metodo 1: Via SSH (consigliato)

### Step 1: Trasferisci file

**Da Windows PowerShell:**
```powershell
# Crea archivio
Compress-Archive -Path remote_ur_control,ros2_bridge_fixed.py -DestinationPath deploy.zip -Force

# Trasferisci
scp deploy.zip lab@192.168.10.191:~/
```

**Oppure usa WinSCP/FileZilla:**
- Host: 192.168.10.191
- User: lab
- Password: easybot
- Trasferisci: `remote_ur_control/` e `ros2_bridge_fixed.py`

### Step 2: SSH sull'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

### Step 3: Estrai e setup

```bash
cd ~
mkdir -p MekoAiAccelerator
cd MekoAiAccelerator

# Se hai trasferito zip:
unzip ~/deploy.zip

# Oppure se hai trasferito direttamente:
# I file sono già qui

# Installa dipendenze
python3 -m pip install --user flask

# Verifica ROS2
source /opt/ros/humble/setup.bash
ros2 --version
```

### Step 4: Avvia ROS2 Driver (se disponibile)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # se installato

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Sul Teach Pendant**: Avvia programma External Control (IP: 192.168.10.191, Porta: 50002)

### Step 5: Avvia Web Interface

```bash
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

python3 -m remote_ur_control.web_interface
```

### Step 6: Testa

Apri browser: `http://192.168.10.191:8080`

## Metodo 2: Via Git (se repo su GitHub)

```bash
ssh lab@192.168.10.191
cd ~
git clone <your-repo-url> MekoAiAccelerator
cd MekoAiAccelerator
git checkout feature/remote-ur-control

# Poi segui Step 3-6 sopra
```

## Verifica Funzionamento

### Test ROS2

```bash
# Terminale 1: Avvia driver ROS2
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194

# Terminale 2: Verifica topic
ros2 topic list | grep servo
ros2 topic echo /servo_node/delta_twist_cmds

# Terminale 3: Avvia web interface
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
python3 -m remote_ur_control.web_interface
```

### Test Web Interface

1. Apri `http://192.168.10.191:8080`
2. Muovi joystick
3. Verifica che vedi messaggi in Terminale 2 (`ros2 topic echo`)

## Troubleshooting

**ROS2 non disponibile?**
- OK, la web interface userà fallback socket
- Funziona ma meno fluido

**Porta 8080 occupata?**
```bash
export WEB_PORT=8081
```

**Errori import?**
```bash
cd ~/MekoAiAccelerator
python3 -m pip install --user flask
```


