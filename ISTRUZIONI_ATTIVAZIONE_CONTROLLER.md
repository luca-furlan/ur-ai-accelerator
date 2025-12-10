# 🎯 ISTRUZIONI: Attivazione Controller

## 🔴 PROBLEMA TROVATO

**Il `forward_velocity_controller` è INACTIVE!**

Questo è il motivo per cui il robot non si muove anche se i messaggi vengono pubblicati.

---

## ✅ SOLUZIONE

### Metodo 1: Usa ros2 control CLI (SE DISPONIBILE)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Installa se non presente
sudo apt install ros-humble-ros2controlcli

# Attiva controller
ros2 control switch_controllers \
    --activate forward_velocity_controller \
    --deactivate scaled_joint_trajectory_controller \
    --strictness 1
```

---

### Metodo 2: Usa ros2 service call (FORMATO CORRETTO)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /controller_manager/switch_controllers \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}"
```

**NOTA**: Il servizio si chiama `SwitchController` (singolare), non `SwitchControllers`!

---

### Metodo 3: Script Python

```bash
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 attiva_controller.py
```

---

## 🔍 VERIFICA

Dopo l'attivazione:

```bash
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | \
    grep -A 1 "forward_velocity_controller" | grep state
```

**Dovresti vedere:**
```
state='active' ✅
```

---

## 🧪 TEST MOVIMENTO

Dopo aver attivato il controller:

```bash
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

**IL ROBOT DOVREBBE MUOVERSI!**

---

## ⚠️ IMPORTANTE

1. **Il driver ROS2 DEVE essere in esecuzione** prima di attivare il controller
2. **Il robot DEVE essere in RUNNING e PLAYING**
3. **External Control DEVE essere attivo**

---

## 🚀 PROCEDURA COMPLETA

1. **Verifica driver ROS2 attivo:**
   ```bash
   ps aux | grep ur_ros2_control
   ```

2. **Se non attivo, avvialo:**
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py \
       ur_type:=ur5e \
       robot_ip:=192.168.10.194 \
       launch_rviz:=false
   ```

3. **Attiva controller:**
   ```bash
   ros2 service call /controller_manager/switch_controllers \
       controller_manager_msgs/srv/SwitchController \
       "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}"
   ```

4. **Test movimento:**
   ```bash
   timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
       std_msgs/msg/Float64MultiArray \
       '{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}'
   ```

5. **Se funziona, prova joystick nella web interface!**

---

## ✅ RISULTATO ATTESO

- ✅ Controller `forward_velocity_controller` attivo
- ✅ Robot risponde ai comandi velocità
- ✅ Joystick funziona!

