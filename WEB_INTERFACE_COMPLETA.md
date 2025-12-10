# 🎮 Web Interface Completa - Controllo Automatico Sistema

## ✅ COSA HO FATTO

Ho creato una **web interface completa** che gestisce tutto automaticamente:

1. ✅ **Avvio Driver ROS2** - Pulsante nella web interface
2. ✅ **Switch Controller** - Cambia tra forward_velocity e scaled_joint_trajectory
3. ✅ **Verifica Stato** - Mostra stato driver, controller, robot, porta 50002
4. ✅ **Controllo Robot** - Joystick e comandi come prima

---

## 🚀 COME USARE

### 1. Avvia Web Interface (UN SOLO COMANDO!)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

### 2. Apri Browser

```
http://192.168.10.191:8080
```

### 3. Usa il Pannello "🚀 Sistema Robot - Controllo Completo"

Vedrai un pannello giallo in alto con:

- **Driver ROS2**: Stato (Attivo/Fermo)
- **Controller**: Quale controller è attivo
- **Robot Mode**: Modalità robot
- **Porta 50002**: Se è aperta

**Pulsanti:**
- **▶️ Avvia Driver ROS2** - Avvia il driver in background
- **🔄 Switch Controller** - Cambia controller automaticamente
- **⏹️ Ferma Driver** - Ferma il driver

---

## 📋 PROCEDURA COMPLETA

### Passo 1: Sul Teach Pendant
1. Assicurati che il robot sia in **Remote Control**
2. Avvia programma con **External Control** (porta 50002)
3. Lascia in **PLAYING**

### Passo 2: Dalla Web Interface
1. Apri `http://192.168.10.191:8080`
2. Clicca **"▶️ Avvia Driver ROS2"**
3. Aspetta 5-10 secondi
4. Clicca **"🔄 Switch Controller"** (attiva forward_velocity_controller)
5. Usa i joystick per muovere il robot!

---

## 🔧 FIX SEGMENTATION FAULT

Ho temporaneamente impostato `forward_velocity_controller` come default per evitare il segmentation fault con `scaled_joint_trajectory_controller`.

**Nel codice:**
```python
# ros2_bridge_fixed.py
self._use_trajectory_control = False  # Usa forward_velocity_controller (più stabile)
```

**Per usare scaled_joint_trajectory_controller:**
1. Avvia driver ROS2 dalla web interface
2. Clicca "Switch Controller" (attiva scaled_joint_trajectory_controller)
3. Se crasha, torna a forward_velocity_controller

---

## 🎯 VANTAGGI

- ✅ **Un solo comando** per avviare tutto
- ✅ **Tutto dalla web interface** - niente terminali multipli
- ✅ **Stato visibile** - vedi subito cosa è attivo
- ✅ **Controllo completo** - avvia, ferma, switch controller

---

## ⚠️ NOTE IMPORTANTI

1. **Il driver ROS2 viene avviato in background** - i log sono in `/tmp/ros2_driver.log`
2. **Per vedere i log del driver:**
   ```bash
   tail -f /tmp/ros2_driver.log
   ```
3. **Per fermare tutto:**
   - Clicca "⏹️ Ferma Driver" nella web interface
   - OPPURE: `pkill -f ur_ros2_control_node`

---

## 🐛 DEBUG

Se qualcosa non funziona:

1. **Verifica driver ROS2:**
   ```bash
   ps aux | grep ur_ros2_control_node
   ```

2. **Verifica controller:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
   ```

3. **Verifica porta 50002:**
   ```bash
   netstat -tuln | grep 50002
   ```

---

## ✅ RISULTATO

Ora puoi:
- ✅ Avviare tutto dalla web interface
- ✅ Vedere lo stato in tempo reale
- ✅ Switch controller con un click
- ✅ Controllare il robot con i joystick
- ✅ **Tutto senza aprire terminali multipli!**

