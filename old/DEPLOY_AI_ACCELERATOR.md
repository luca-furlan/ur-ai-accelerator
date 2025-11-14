# Deploy su AI Accelerator

## Trasferimento File

### Opzione 1: Script automatico (da Windows con Git Bash/WSL)

```bash
bash deploy_to_ai_accelerator.sh
```

### Opzione 2: Manuale via SCP

```bash
# Da Windows PowerShell o Git Bash
scp -r remote_ur_control lab@192.168.10.191:~/MekoAiAccelerator/
scp ros2_bridge_fixed.py lab@192.168.10.191:~/MekoAiAccelerator/
```

### Opzione 3: Git (se hai repo su GitHub)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
git pull origin feature/remote-ur-control
```

## Setup su AI Accelerator

### 1. SSH sull'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

### 2. Installa dipendenze

```bash
cd ~/MekoAiAccelerator
python3 -m pip install --user flask
# Oppure se hai venv:
# python3 -m venv ~/.venvs/ur-remote
# source ~/.venvs/ur-remote/bin/activate
# pip install -r remote_ur_control/requirements.txt
```

### 3. Verifica ROS2 (opzionale ma consigliato)

```bash
source /opt/ros/humble/setup.bash
ros2 --version

# Se hai il driver UR installato:
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep ur_robot_driver
```

### 4. Avvia ROS2 Driver (se disponibile)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # se installato

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Sul Teach Pendant**: Avvia programma con External Control (IP: 192.168.10.191, Porta: 50002)

### 5. Avvia Web Interface

```bash
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

python3 -m remote_ur_control.web_interface
```

### 6. Accedi da browser

Apri: `http://192.168.10.191:8080`

## Test

1. Verifica che ROS2 sia disponibile:
   ```bash
   ros2 topic list | grep servo
   ```

2. Verifica che la web interface risponda:
   ```bash
   curl http://localhost:8080/api/config
   ```

3. Muovi il joystick nella web interface
4. Verifica che i comandi arrivino:
   ```bash
   ros2 topic echo /servo_node/delta_twist_cmds
   ```

## Troubleshooting

**ROS2 non disponibile?**
- La web interface userà fallback socket (meno fluido ma funziona)

**Porta 8080 occupata?**
- Cambia porta: `export WEB_PORT=8081`

**Permessi?**
- Usa `--user` per pip install o crea venv


