# 🔧 SOLUZIONE: URScript send failed (timed out)

## Problema Identificato

Quando vedi:
```
URScript send failed (timed out), reconnecting...
⚠️ ROS2 not available (rclpy not found)
```

**Causa:**
1. ROS2 non è sourceato → `rclpy` non disponibile
2. Web interface usa fallback socket diretto (porta 30002)
3. **MA**: Quando driver ROS2 è attivo e robot è in External Control, la porta 30002 NON accetta comandi socket diretti
4. Il robot può ricevere comandi SOLO via ROS2 quando è in External Control

---

## ✅ SOLUZIONE

### Opzione 1: Usa lo script aggiornato (CONSIGLIATO)

Lo script `avvia_web_interface_joystick.sh` è stato aggiornato per sourceare ROS2 automaticamente:

```bash
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

Lo script ora:
- ✅ Sourcea ROS2 Humble automaticamente
- ✅ Sourcea workspace ROS2 automaticamente
- ✅ Abilita ROS2 bridge nella web interface
- ✅ Usa ROS2 topic invece di socket diretto

---

### Opzione 2: Avvio manuale con ROS2

Se vuoi avviare manualmente:

```bash
ssh lab@192.168.10.191

# IMPORTANTE: Sourcea ROS2 PRIMA di avviare web interface
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Poi avvia web interface
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8080
cd ~/MekoAiAccelerator
python3 -m remote_ur_control.web_interface
```

---

## 🔍 Verifica che Funzioni

Dopo aver avviato con ROS2 sourceato, dovresti vedere:

**✅ Messaggi positivi:**
```
✅ ROS2 bridge initialized - publishing at 125Hz
🌐 Starting web interface on http://0.0.0.0:8080
```

**❌ NON dovresti vedere:**
```
⚠️ ROS2 not available (rclpy not found)
URScript send failed (timed out)
```

---

## 📋 Checklist

Prima di usare i joystick:

- [ ] Driver ROS2 attivo (`ur_ros2_control_node` in esecuzione)
- [ ] Robot connesso via External Control (programma in PLAYING)
- [ ] ROS2 sourceato PRIMA di avviare web interface
- [ ] Web interface avviata con ROS2 disponibile
- [ ] Nessun errore "rclpy not found"
- [ ] Nessun errore "URScript send failed"

---

## 🎯 Come Funziona Ora

1. **Con ROS2 disponibile:**
   - Web interface usa ROS2 bridge
   - Pubblica su `/forward_velocity_controller/commands`
   - Driver ROS2 traduce in comandi robot
   - ✅ Funziona quando robot è in External Control

2. **Senza ROS2 (fallback):**
   - Web interface usa socket diretto (porta 30002)
   - ❌ NON funziona quando driver ROS2 è attivo
   - ✅ Funziona solo se driver ROS2 NON è attivo

---

## 🐛 Se Ancora Non Funziona

1. **Verifica ROS2 disponibile:**
   ```bash
   python3 -c "import rclpy; print('OK')"
   ```
   Dovrebbe stampare "OK"

2. **Verifica topic ROS2:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 topic list | grep forward_velocity
   ```
   Dovresti vedere `/forward_velocity_controller/commands`

3. **Verifica pubblicazione:**
   ```bash
   ros2 topic echo /forward_velocity_controller/commands
   ```
   Muovi il joystick → dovresti vedere messaggi

4. **Verifica driver ROS2:**
   ```bash
   ps aux | grep ur_ros2_control_node
   ```
   Dovrebbe essere in esecuzione

---

## ✅ Risultato Atteso

Dopo aver applicato la soluzione:
- ✅ Nessun errore "rclpy not found"
- ✅ Nessun errore "URScript send failed"
- ✅ Joystick funziona e robot si muove
- ✅ Movimenti fluidi e controllati







