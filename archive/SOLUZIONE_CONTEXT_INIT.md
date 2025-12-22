# 🔧 SOLUZIONE: Context.init() must only be called once

## Problema

Errore:
```
RuntimeError: Context.init() must only be called once
```

**Causa:**
- ROS2 è già inizializzato (probabilmente dal driver ROS2)
- Il bridge ROS2 cerca di inizializzarlo di nuovo
- `rclpy.init()` può essere chiamato solo una volta per processo

---

## ✅ SOLUZIONE APPLICATA

Ho modificato `ros2_bridge_fixed.py` per gestire correttamente il caso in cui ROS2 è già inizializzato:

```python
try:
    if not rclpy.ok():
        rclpy.init()
except RuntimeError as e:
    if 'must only be called once' in str(e) or 'already initialized' in str(e).lower():
        # ROS2 già inizializzato - va bene, continuiamo
        pass
    else:
        raise
```

---

## 🚀 COSA FARE ORA

1. **Ferma la web interface** (se è ancora in esecuzione):
   ```bash
   pkill -f web_interface
   ```

2. **Riavvia la web interface:**
   ```bash
   cd ~/MekoAiAccelerator
   ./avvia_web_interface_joystick.sh
   ```

3. **Verifica che non ci siano più errori:**
   - NON dovresti vedere: `Context.init() must only be called once`
   - Dovresti vedere: `✅ ROS2 bridge initialized`

---

## 🔍 VERIFICA

Dopo il riavvio, controlla i log:

**✅ Messaggi positivi:**
```
✅ ROS2 Humble configurato
✅ Workspace ROS2 configurato
✅ ROS2 bridge initialized
🔄 Starting publish loop at 125Hz...
```

**❌ NON dovresti vedere:**
```
RuntimeError: Context.init() must only be called once
⚠️ Failed to initialize ROS2
```

---

## 📋 NOTE

- ROS2 può essere inizializzato solo una volta per processo
- Se il driver ROS2 è già in esecuzione, ROS2 è già inizializzato
- Il bridge ROS2 ora gestisce correttamente questo caso
- Il nodo può essere creato anche se ROS2 è già inizializzato

---

## 🐛 Se Ancora Non Funziona

1. **Verifica che ROS2 sia disponibile:**
   ```bash
   python3 -c "import rclpy; print('OK')"
   ```

2. **Verifica che il driver ROS2 sia attivo:**
   ```bash
   ps aux | grep ur_ros2_control_node
   ```

3. **Riavvia tutto:**
   ```bash
   # Ferma tutto
   pkill -f 'ur_robot_driver|web_interface'
   
   # Riavvia driver ROS2
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false &
   
   # Riavvia web interface
   cd ~/MekoAiAccelerator
   ./avvia_web_interface_joystick.sh
   ```







