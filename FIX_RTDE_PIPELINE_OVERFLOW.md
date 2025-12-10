# 🔧 FIX: Pipeline Producer Overflowed - RTDE Data Pipeline

## ❌ Problema

```
[ERROR] [UR_Client_Library:]: Pipeline producer overflowed! <RTDE Data Pipeline>
```

Questo errore indica che il buffer RTDE si sta riempiendo più velocemente di quanto riesca a essere processato.

## 🔍 Cause Comuni

1. **Troppi subscriber RTDE** - Più processi stanno leggendo RTDE contemporaneamente
2. **Frequenza RTDE troppo alta** - Il robot invia dati troppo velocemente
3. **Processore sovraccarico** - Il sistema non riesce a processare i dati abbastanza velocemente
4. **Connessioni RTDE multiple** - Più client RTDE connessi contemporaneamente

## ✅ Soluzioni

### SOLUZIONE 1: Chiudi altre connessioni RTDE

Verifica se ci sono altri processi che usano RTDE:

```bash
# Verifica processi RTDE
ps aux | grep rtde
ps aux | grep ur_rtde

# Chiudi web interface se sta usando RTDE
pkill -f web_interface

# Chiudi altri script Python che usano RTDE
pkill -f rtde
```

### SOLUZIONE 2: Riduci frequenza RTDE nel launch file

Modifica il launch file per ridurre la frequenza RTDE:

```bash
# Crea un launch file custom
nano ~/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/launch/ur_control_custom.launch.py
```

Aggiungi parametro per ridurre frequenza RTDE:

```python
rtde_input_recipe_frequency = 125.0  # Riduci a 50 o 25
rtde_output_recipe_frequency = 125.0  # Riduci a 50 o 25
```

### SOLUZIONE 3: Usa solo un client RTDE alla volta

**IMPORTANTE**: Solo UN processo alla volta può usare RTDE!

- Se il driver UR ROS2 è in esecuzione → NON usare RTDE nella web interface
- Se la web interface usa RTDE → NON avviare il driver UR ROS2

### SOLUZIONE 4: Verifica configurazione robot

Sul teach pendant, verifica che:
- Solo UN programma con External Control sia in PLAYING
- Non ci siano altri programmi che usano RTDE

### SOLUZIONE 5: Riavvia tutto pulito

```bash
# 1. Ferma tutto
pkill -f ur_robot_driver
pkill -f web_interface
pkill -f rtde

# 2. Aspetta 2 secondi
sleep 2

# 3. Avvia SOLO driver UR ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

## 🎯 Soluzione Raccomandata

**Per controllo ROS2:**
1. Ferma web interface (usa RTDE)
2. Avvia SOLO driver UR ROS2
3. Usa ROS2 topics per controllo

**Per controllo socket (senza ROS2):**
1. Ferma driver UR ROS2 (usa RTDE)
2. Avvia web interface
3. Usa socket diretto (porta 30002)

## ⚠️ REGOLA D'ORO

**RTDE può essere usato da UN SOLO processo alla volta!**

- Driver UR ROS2 usa RTDE → NON usare RTDE altrove
- Web interface usa RTDE → NON avviare driver UR ROS2




