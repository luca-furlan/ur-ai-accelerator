# Test su AI Accelerator - Istruzioni Rapide

## 1. SSH sull'AI Accelerator

```bash
ssh lab@192.168.10.191
# Password: easybot
```

## 2. Estrai e Setup

```bash
cd ~
unzip deploy.zip -d MekoAiAccelerator
cd MekoAiAccelerator

# Rendi eseguibile lo script di test
chmod +x ../test_on_ai_accelerator.sh
../test_on_ai_accelerator.sh
```

## 3. Avvia ROS2 Driver (se disponibile)

**Terminale 1:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # se installato

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Sul Teach Pendant**: Avvia programma External Control (IP: 192.168.10.191, Porta: 50002)

## 4. Verifica ROS2 (opzionale)

**Terminale 2:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Verifica topic
ros2 topic list | grep servo

# Monitora comandi (in un altro terminale)
ros2 topic echo /servo_node/delta_twist_cmds
```

## 5. Avvia Web Interface

**Terminale 3:**
```bash
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

python3 -m remote_ur_control.web_interface
```

Dovresti vedere:
```
✅ ROS2 bridge initialized  (se ROS2 disponibile)
⚠️ ROS2 bridge not available - using socket fallback  (se ROS2 non disponibile)
 * Running on http://0.0.0.0:8080
```

## 6. Testa dal Browser

Apri: `http://192.168.10.191:8080`

1. Muovi il joystick
2. Se ROS2 è attivo, vedi messaggi in Terminale 2 (`ros2 topic echo`)
3. Il robot dovrebbe muoversi fluidamente

## Troubleshooting

**"ROS2 bridge not available"?**
- OK, userà fallback socket
- Funziona ma meno fluido

**"Failed to initialize ROS2"?**
- Verifica: `ros2 --version`
- Verifica: `source /opt/ros/humble/setup.bash`

**Robot non si muove?**
- Verifica che programma sul teach pendant sia PLAYING
- Verifica connessione: `ping 192.168.10.194`
- Controlla errori nella console web interface


