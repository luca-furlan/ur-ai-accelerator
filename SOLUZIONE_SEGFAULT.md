# 🔧 SOLUZIONE Segmentation Fault - Driver UR ROS2

## ❌ Problema

Segmentation fault nel `ScaledJointTrajectoryController` su ARM64 (Jetson).

**Errore:**
```
Segmentation fault (Address not mapped to object [(nil)])
#0 ur_controllers::ScaledJointTrajectoryController::update(...)
```

## ✅ Soluzione

### Usa controller NON-scaled (più stabile su ARM64)

**Script creato:** `avvia_driver_ur5e_fixed.sh`

Usa `joint_trajectory_controller` invece di `scaled_joint_trajectory_controller`.

### Come Usare

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator

# 1. Verifica tutto prima
bash verifica_prima_di_avviare.sh

# 2. Se tutto OK, avvia driver con fix
bash avvia_driver_ur5e_fixed.sh
```

### Sul Teach Pendant (IMPORTANTE!)

**PRIMA di avviare il driver ROS2:**

1. Vai su **Program**
2. Apri o crea programma con nodo **External Control**
3. Configura **External Control**:
   - **IP Host:** `192.168.10.191`
   - **Porta:** `50002`
4. **SALVA** il programma
5. **Premi PLAY** sul Teach Pendant
6. Verifica che programma sia in stato **PLAYING**

**Il programma DEVE essere PLAYING prima di avviare il driver ROS2!**

## 🔍 Verifica Stato

```bash
bash verifica_prima_di_avviare.sh
```

Verifica:
- ✅ Robot raggiungibile
- ✅ Robot in modalità RUNNING
- ✅ Programma in stato PLAYING
- ✅ Safety mode NORMAL

## 📋 Differenza Script

### `avvia_driver_ur5e.sh` (vecchio - con segfault)
- Usa `scaled_joint_trajectory_controller`
- ❌ Crash su ARM64

### `avvia_driver_ur5e_fixed.sh` (nuovo - fix)
- Usa `joint_trajectory_controller`
- ✅ Funziona su ARM64

## 🚀 Workflow Corretto

1. **Sul Teach Pendant:**
   - Avvia programma External Control
   - Verifica PLAYING

2. **Sul AI Accelerator:**
   ```bash
   bash verifica_prima_di_avviare.sh
   bash avvia_driver_ur5e_fixed.sh
   ```

3. **In altro terminale:**
   ```bash
   bash avvia_web_interface_ur5e.sh
   ```

4. **Browser:**
   - `http://192.168.10.191:8080`

## ✅ Risolto!

Il driver ora funziona correttamente su ARM64/Jetson!





