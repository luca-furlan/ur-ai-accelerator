# 🔧 FIX CRITICO: Segmentation Fault con scaled_joint_trajectory_controller

## 🔴 PROBLEMA IDENTIFICATO

Il driver ROS2 **CRASHA** con segmentation fault quando attiva `scaled_joint_trajectory_controller`:

```
[ERROR] Pipeline producer overflowed! <RTDE Data Pipeline>
Segmentation fault (Address not mapped to object [(nil)])
[ERROR] [ur_ros2_control_node-1]: process has died [pid 10064, exit code -11]
```

**Causa**: Il `scaled_joint_trajectory_controller` causa un crash quando le posizioni non sono inizializzate correttamente o quando c'è overflow RTDE.

---

## ✅ SOLUZIONE APPLICATA

**Cambiato a `forward_velocity_controller`** che:
- ✅ Non causa segmentation fault
- ✅ Funziona senza bisogno di posizioni inizializzate
- ✅ Più semplice e diretto
- ✅ Funzionava prima quando tutto andava bene

---

## 🔄 COSA FARE ORA

### 1. Ferma il Driver ROS2 Corrente

Nel terminale dove è crashato:
- Premi **CTRL+C** per fermare
- OPPURE:
```bash
pkill -f "ur_robot_driver|ur_control"
```

### 2. Riavvia Driver ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

### 3. Attiva forward_velocity_controller

**In un nuovo terminale:**

```bash
ssh lab@192.168.10.191
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Attiva forward_velocity_controller
ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}"
```

### 4. Avvia Web Interface

```bash
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

---

## ✅ RISULTATO ATTESO

- ✅ Driver ROS2 non crasha più
- ✅ `forward_velocity_controller` attivo
- ✅ Robot si muove correttamente
- ✅ Nessun segmentation fault

---

## ⚠️ NOTA IMPORTANTE

**NON usare `scaled_joint_trajectory_controller`** finché non risolviamo il problema del segmentation fault. Usa sempre `forward_velocity_controller` per ora.







