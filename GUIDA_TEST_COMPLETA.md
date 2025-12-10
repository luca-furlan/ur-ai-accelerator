# 🧪 Guida Completa ai Test del Sistema

## Panoramica

Questa guida descrive come verificare e testare tutti i componenti del sistema AI Accelerator per Universal Robots.

## Struttura Test

```
MekoAiAccelerator/
├── test_sistema_completo.py          # Verifica completa installazione
├── test/
│   ├── run_all_tests.py              # Esegue tutti i test
│   ├── test_connettivita_robot.py    # Test connessione robot
│   ├── test_ros2_driver.py           # Test ROS2 driver UR
│   ├── test_camera_orbbec.py         # Test camera Orbbec
│   ├── test_mujoco.py                # Test MuJoCo
│   ├── test_ai_components.py         # Test componenti AI
│   └── test_web_interface.py          # Test web interface
└── GUIDA_TEST_COMPLETA.md            # Questa guida
```

## Quick Start

### 1. Verifica Completa Sistema

Esegue una verifica completa di tutti i componenti installati:

```bash
cd ~/MekoAiAccelerator
python3 test_sistema_completo.py
```

**Cosa verifica:**
- ✅ Sistema operativo (Ubuntu, Python, ROS2)
- ✅ Driver UR e interfacce
- ✅ MuJoCo e modelli
- ✅ Camera Orbbec
- ✅ Componenti AI (YOLO, OpenCV, Open3D)
- ✅ MoveIt2
- ✅ Componenti custom
- ✅ Connettività di rete
- ✅ ROS2 runtime (topics, nodi)

**Output:**
- Report dettagliato con ✅/❌/⚠️ per ogni componente
- File JSON con risultati: `~/test_sistema_results.json`

### 2. Eseguire Tutti i Test Funzionali

Esegue tutti i test funzionali in sequenza:

```bash
cd ~/MekoAiAccelerator
python3 test/run_all_tests.py
```

**Cosa testa:**
- Connettività robot (socket, RTDE)
- ROS2 driver (topics, nodi)
- Camera Orbbec (topics, connessione)
- MuJoCo (installazione, modelli)
- Componenti AI (YOLO, OpenCV, Open3D)
- Web Interface (import, inizializzazione)

### 3. Test Singoli

Eseguire test specifici:

```bash
# Test connettività robot
python3 test/test_connettivita_robot.py

# Test ROS2 driver
python3 test/test_ros2_driver.py

# Test camera Orbbec
python3 test/test_camera_orbbec.py

# Test MuJoCo
python3 test/test_mujoco.py

# Test componenti AI
python3 test/test_ai_components.py

# Test web interface
python3 test/test_web_interface.py
```

## Test Dettagliati

### Test Connettività Robot

**File:** `test/test_connettivita_robot.py`

**Cosa testa:**
- Connessione socket al robot (porta 30002)
- Connessione RTDE al robot (porta 30004)

**Prerequisiti:**
- Robot acceso e raggiungibile
- IP robot: `192.168.10.194` (modificabile nel codice)

**Esempio output:**
```
✅ Socket connesso
✅ RTDE connesso
```

### Test ROS2 Driver

**File:** `test/test_ros2_driver.py`

**Cosa testa:**
- Disponibilità ROS2
- Topics ROS2 disponibili
- Nodi ROS2 disponibili
- Topics UR specifici
- Pubblicazione dati su topics

**Prerequisiti:**
- ROS2 Humble installato
- ROS2 daemon avviato: `ros2 daemon start`
- (Opzionale) Driver UR avviato per test completi

**Per avviare driver UR:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

**Esempio output:**
```
✅ ROS2 disponibile
✅ Topics disponibili: 45
✅ Topics UR rilevati: 12
  - /joint_states
  - /servo_node/delta_twist_cmds
  ...
```

### Test Camera Orbbec

**File:** `test/test_camera_orbbec.py`

**Cosa testa:**
- Pacchetto ROS2 Orbbec installato
- Topics camera disponibili
- Connessione camera funzionante
- SDK Python Orbbec (opzionale)

**Prerequisiti:**
- OrbbecSDK_ROS2 installato nel workspace ROS2
- (Opzionale) Camera avviata per test completi

**Per avviare camera:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch orbbec_camera gemini_330_series.launch.py
```

**Esempio output:**
```
✅ Pacchetto ROS2 Orbbec installato
✅ Topics Orbbec disponibili: 8
  - /camera/color/image_raw
  - /camera/depth/image_raw
  ...
✅ Camera connessa e funzionante
```

### Test MuJoCo

**File:** `test/test_mujoco.py`

**Cosa testa:**
- MuJoCo installato
- MuJoCo viewer disponibile
- MuJoCo Menagerie presente
- Modelli UR5e/UR10e presenti e caricabili

**Prerequisiti:**
- MuJoCo installato: `pip install mujoco`
- (Opzionale) MuJoCo Menagerie clonato

**Per installare MuJoCo Menagerie:**
```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
```

**Esempio output:**
```
✅ MuJoCo installato (versione: 3.1.0)
✅ MuJoCo viewer disponibile
✅ MuJoCo Menagerie trovato
✅ Modello UR5e presente
   ✅ Modello UR5e caricabile
```

### Test Componenti AI

**File:** `test/test_ai_components.py`

**Cosa testa:**
- YOLOv8 installato e funzionante
- OpenCV installato e funzionante
- Open3D installato e funzionante

**Prerequisiti:**
- Pacchetti installati:
  - `pip install ultralytics` (YOLOv8)
  - `pip install opencv-python` (OpenCV)
  - `pip install open3d` (Open3D)

**Esempio output:**
```
✅ YOLOv8 installato (versione: 8.0.0)
   ✅ YOLOv8 funzionante
✅ OpenCV installato (versione: 4.8.0)
   ✅ OpenCV funzionante
✅ Open3D installato (versione: 0.18.0)
   ✅ Open3D funzionante
```

### Test Web Interface

**File:** `test/test_web_interface.py`

**Cosa testa:**
- File componenti presenti
- Import moduli funzionanti
- Inizializzazione ROS2 bridge
- Porta web disponibile

**Prerequisiti:**
- File progetto presenti
- ROS2 disponibile (opzionale per alcuni test)

**Esempio output:**
```
✅ web_interface.py presente
✅ ros2_bridge_fixed.py presente
✅ Web interface importabile
✅ ROS2 bridge importabile
✅ ROS2 bridge inizializzato
✅ Porta 8080 disponibile
```

## Interpretazione Risultati

### Simboli

- ✅ **Verde**: Componente installato e funzionante
- ❌ **Rosso**: Componente mancante o non funzionante (richiesto)
- ⚠️ **Giallo**: Componente opzionale o da verificare

### Stati Sistema

**Sistema Pronto:**
- Tutti i componenti richiesti ✅
- Nessun errore critico ❌

**Sistema Quasi Pronto:**
- 1-2 componenti mancanti
- Componenti opzionali mancanti

**Sistema Incompleto:**
- 3+ componenti richiesti mancanti
- Componenti critici non funzionanti

## Troubleshooting

### ROS2 Non Disponibile

```bash
# Verifica installazione
ros2 --version

# Se non installato
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS2
source /opt/ros/humble/setup.bash
```

### Driver UR Non Trovato

```bash
# Verifica workspace
ls ~/ros2_ws/src/Universal_Robots_ROS2_Driver

# Se mancante, clonare
cd ~/ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

# Compilare
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Camera Orbbec Non Rilevata

```bash
# Verifica pacchetto ROS2
ros2 pkg list | grep orbbec

# Se mancante, clonare
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Compilare
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### MuJoCo Non Installato

```bash
# Installare MuJoCo
pip install mujoco

# Installare Menagerie (opzionale)
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
```

## Workflow Consigliato

### 1. Verifica Iniziale

```bash
# Verifica completa sistema
python3 test_sistema_completo.py
```

### 2. Test Funzionali

```bash
# Esegui tutti i test
python3 test/run_all_tests.py
```

### 3. Test Specifici

```bash
# Test componente specifico
python3 test/test_<componente>.py
```

### 4. Verifica Dopo Modifiche

```bash
# Dopo installazione/modifica componente
python3 test/test_<componente>.py
```

## File Risultati

### test_sistema_results.json

File JSON con risultati completi della verifica sistema:
- Stato di ogni componente
- Versioni installate
- Path file/directory
- Errori rilevati

**Location:** `~/test_sistema_results.json`

## Note Importanti

1. **ROS2 Daemon**: Alcuni test richiedono ROS2 daemon avviato:
   ```bash
   ros2 daemon start
   ```

2. **Source ROS2**: Prima di eseguire test ROS2:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **Robot Connesso**: Test connettività richiedono robot acceso e raggiungibile.

4. **Camera Connessa**: Test camera richiedono camera fisicamente connessa.

## Supporto

Per problemi o domande:
1. Verificare output test per errori specifici
2. Controllare prerequisiti per ogni test
3. Verificare log ROS2: `ros2 topic echo /rosout`

---

**Ultimo aggiornamento:** 2025-01-XX
**Versione sistema:** AI Accelerator 1.1 + PolyScope X 10.8





