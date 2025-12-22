# 🎯 Metodo Guida Pratica - Muovere UR5e con ROS2

**Fonte**: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db  
**Autore**: Shawn Armstrong

Questa cartella contiene tutti i file e gli script per configurare e controllare il robot UR5e usando ROS2 seguendo una guida pratica testata.

## 📁 Contenuto Cartella

- `SETUP_COMPLETO.sh` - Script principale per setup completo
- `TEST_MOVIMENTO.sh` - Script per test movimento robot
- `README.md` - Questa guida
- `externalcontrol-1.0.5.urcap` - File URCap (verrà scaricato automaticamente)

## 🚀 Quick Start

### 1. Setup Completo

```bash
cd ~/MekoAiAccelerator/metodo_guida_pratica
chmod +x SETUP_COMPLETO.sh
./SETUP_COMPLETO.sh
```

Lo script ti guiderà attraverso:
- Download URCap
- Installazione sul Teach Pendant
- Configurazione programma
- Verifica configurazione
- Avvio driver ROS2

### 2. Test Movimento

In un altro terminale (dopo che il driver è avviato):

```bash
cd ~/MekoAiAccelerator/metodo_guida_pratica
chmod +x TEST_MOVIMENTO.sh
./TEST_MOVIMENTO.sh
```

## 📋 Passi Dettagliati

### PASSO 1: Scarica URCap

Lo script scarica automaticamente `externalcontrol-1.0.5.urcap` da GitHub.

### PASSO 2: Installa URCap sul Robot

1. Copia `externalcontrol-1.0.5.urcap` su USB
2. Inserisci USB nel Teach Pendant
3. Installation → URCaps → '+' → Seleziona file → Riavvia

### PASSO 3: Configura Programma

1. Crea programma con nodo "External Control"
2. IP Host: `192.168.10.191`
3. Porta: `50002`
4. (Opzionale) Script iniziale: `movej([0, 0, 0, 0, 0, 0], a=1.0, v=1.0)`
5. Salva e avvia (PLAY)

### PASSO 4: Avvia Driver ROS2

Lo script avvia automaticamente con:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**⚠️ NOTA**: `launch_rviz:=false` evita conflitti di rete secondo la guida.

### PASSO 5: Test Movimento

```bash
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
```

## ✅ Verifica

### Verifica Controller

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

Dovresti vedere:
- `scaled_joint_trajectory_controller [active]`
- `forward_velocity_controller [active]`
- `joint_state_broadcaster [active]`

### Verifica Topic

```bash
ros2 topic list
ros2 topic echo /joint_states
```

## 🔧 Troubleshooting

### Porta 50002 chiusa

1. Verifica programma in PLAYING sul Teach Pendant
2. Verifica nodo External Control presente
3. Verifica IP: `192.168.10.191`
4. Verifica porta: `50002`
5. STOP e PLAY di nuovo

### Driver non si connette

1. Verifica Remote Control abilitato
2. Verifica connessione: `ping 192.168.10.194`
3. Verifica porta 50002 aperta

### Robot non si muove

1. Verifica driver ROS2 in esecuzione
2. Verifica controller attivi
3. Verifica programma in PLAYING
4. Controlla log driver

## 📚 Riferimenti

- **Guida originale**: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db
- **URCap External Control**: https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap
- **Driver ROS2**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## ⚠️ Note Importanti

1. **launch_rviz:=false**: Evita conflitti di rete
2. **Script iniziale**: Porta robot in posizione iniziale
3. **Programma in PLAYING**: Obbligatorio prima di avviare driver
4. **Remote Control**: Deve essere abilitato

## 🎯 Prossimi Passi

Dopo aver verificato che funziona:
1. Sviluppa comandi personalizzati
2. Integra con altri sistemi ROS2
3. Usa MoveIt2 per traiettorie avanzate










