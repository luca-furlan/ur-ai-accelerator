# UR5e Remote Control - AI Accelerator

Sistema di controllo remoto per robot UR5e tramite web interface.

## 🚀 Quick Start

### 1. Avvia Driver ROS2
```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
export UR_ROBOT_IP=192.168.10.194
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false initial_joint_controller:=forward_velocity_controller
```

### 2. Avvia Web Interface
```bash
# In un altro terminale
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 start_web_interface.py
```

### 3. Accedi alla Web Interface
Apri browser: `http://192.168.10.191:8080`

## 📁 Struttura Repository

```
MekoAiAccelerator/
├── remote_ur_control/          # Codice principale
│   ├── web_interface.py        # Web interface Flask
│   └── remote_ur_controller.py # Controller robot
├── switch_controller.py        # Switch controller robusto
├── attiva_controller.py        # Attiva controller
├── attiva_controller.sh        # Script bash attivazione
├── start_web_interface.py      # Avvia web interface
├── ros2_bridge_fixed.py        # Bridge ROS2
├── archive/                     # File archiviati
└── README.md                   # Questo file
```

## 🔧 Script Essenziali

### Switch Controller
```bash
python3 switch_controller.py forward_velocity_controller scaled_joint_trajectory_controller
```

### Attiva Controller
```bash
python3 attiva_controller.py
# oppure
./attiva_controller.sh
```

## ⚙️ Configurazione

### Variabili Ambiente
- `UR_ROBOT_IP`: IP del robot (default: 192.168.10.194)
- `WEB_PORT`: Porta web interface (default: 8080)
- `WEB_HOST`: Host web interface (default: 0.0.0.0)

## 🐛 Troubleshooting

### ROS2 non disponibile
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Controller non attivo
```bash
python3 attiva_controller.py
```

### Verifica stato controller
```bash
ros2 control list_controllers | grep forward_velocity_controller
```

## 📝 Note

- Il driver ROS2 parte già con `forward_velocity_controller` attivo
- Gli script verificano lo stato prima di fare switch
- Se il controller è già attivo, gli script ritornano successo senza fare nulla
- Script di verifica e documentazione vecchia sono in `archive/`
