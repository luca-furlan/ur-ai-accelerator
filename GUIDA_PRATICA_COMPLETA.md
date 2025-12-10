# 🎯 GUIDA PRATICA COMPLETA - Muovere UR5e con ROS2

**Fonte**: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db  
**Autore**: Shawn Armstrong

## 📋 Panoramica

Questa guida pratica mostra come configurare e muovere un robot UR usando ROS2, basandosi su una guida testata con successo.

## ✅ Requisiti

- Robot UR5e (o altro modello UR)
- ROS2 Humble installato
- Driver UR ROS2 installato
- Connessione di rete tra PC e robot

## 🚀 Passi Completi

### PASSO 1: Scarica URCap External Control

```bash
cd ~/MekoAiAccelerator
./SETUP_SECONDO_GUIDA_PRATICA.sh
```

Lo script scarica automaticamente `externalcontrol-1.0.5.urcap` da GitHub.

**OPPURE manualmente:**
```bash
wget https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.urcap
```

### PASSO 2: Installa URCap sul Robot (Teach Pendant)

1. **Copia il file** `externalcontrol-1.0.5.urcap` su una chiavetta USB
2. **Inserisci la USB** nel Teach Pendant
3. **Sul Teach Pendant**:
   - Vai su: **Installation** → **URCaps**
   - Premi il pulsante **'+'** (Aggiungi)
   - Seleziona il file `externalcontrol-1.0.5.urcap` dalla USB
   - Premi **'Apri'**
   - **Riavvia il robot** quando richiesto

### PASSO 3: Configura Programma sul Teach Pendant

1. **Crea nuovo programma** (o apri `remote_control.urp`)

2. **Aggiungi nodo 'External Control'**:
   - Vai su: **Structure** → **URCaps** → **External Control**
   - Trascina il nodo nel programma

3. **Configura il nodo External Control**:
   - Clicca sul nodo External Control
   - **IP Host**: `192.168.10.191` (IP AI Accelerator)
   - **Porta**: `50002` (default)

4. **Aggiungi script iniziale** (opzionale ma consigliato):
   - Prima del nodo External Control, aggiungi uno script
   - Inserisci: `movej([0, 0, 0, 0, 0, 0], a=1.0, v=1.0)`
   - Questo porta il robot in posizione iniziale

5. **Salva il programma** (es: `ros_control.urp`)

6. **Avvia il programma** (PLAY sul Teach Pendant)

### PASSO 4: Verifica Configurazione

```bash
./SETUP_SECONDO_GUIDA_PRATICA.sh
```

Lo script verifica:
- ✅ Remote Control abilitato
- ✅ Porta 50002 aperta (External Control attivo)

### PASSO 5: Avvia Driver ROS2

Lo script avvia automaticamente il driver con:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**⚠️ NOTA**: Secondo la guida, `launch_rviz:=false` evita conflitti di rete che possono far fallire il driver.

**Aspetta che vedi:**
```
Robot connected to reverse interface. Ready to receive control commands.
```

### PASSO 6: Test Movimento

In un altro terminale:

```bash
./TEST_MOVIMENTO_SECONDO_GUIDA.sh
```

**OPPURE manualmente:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

Questo invierà un **loop infinito di comandi di movimento** al robot.

**Per fermare**: Premi CTRL+C

## 📊 Verifica Controller

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
sudo apt install ros-humble-ros2controlcli  # se non installato
ros2 control list_controllers
```

Dovresti vedere controller attivi come:
- `scaled_joint_trajectory_controller [active]`
- `forward_velocity_controller [active]`
- `joint_state_broadcaster [active]`

## 🔧 Troubleshooting

### Porta 50002 chiusa

1. Verifica che il programma sia in **PLAYING** sul Teach Pendant
2. Verifica che il nodo External Control sia presente nel programma
3. Verifica IP Host: deve essere `192.168.10.191`
4. Verifica porta: deve essere `50002`
5. STOP e poi PLAY di nuovo il programma

### Driver non si connette

1. Verifica Remote Control abilitato: `Settings → System → Remote Control → Enable`
2. Verifica connessione di rete: `ping 192.168.10.194`
3. Verifica che la porta 50002 sia aperta: usa `./SETUP_SECONDO_GUIDA_PRATICA.sh`

### Robot non si muove

1. Verifica che il driver ROS2 sia in esecuzione
2. Verifica che i controller siano attivi: `ros2 control list_controllers`
3. Verifica che il programma sia in PLAYING sul Teach Pendant
4. Controlla i log del driver per errori

## 📚 Riferimenti

- **Guida originale**: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db
- **URCap External Control**: https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap
- **Driver ROS2**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## ⚠️ Note Importanti

1. **launch_rviz:=false**: Secondo la guida, questo evita conflitti di rete
2. **Script iniziale**: Aggiungere `movej([0, 0, 0, 0, 0, 0], a=1.0, v=1.0)` porta il robot in posizione iniziale
3. **Programma in PLAYING**: Il programma External Control DEVE essere in PLAYING prima di avviare il driver
4. **Remote Control**: Deve essere abilitato sul Teach Pendant

## 🎯 Prossimi Passi

Dopo aver verificato che il movimento funziona:

1. Sviluppa i tuoi comandi di movimento personalizzati
2. Integra con altri sistemi ROS2
3. Usa MoveIt2 per pianificazione traiettorie avanzate



