# 🚀 Quick Reference - Test Sistema

## Comandi Rapidi

```bash
# Quick check (verifica rapida)
python3 quick_check_sistema.py

# Verifica completa sistema
python3 test_sistema_completo.py

# Tutti i test funzionali
python3 test/run_all_tests.py

# Test singoli
python3 test/test_connettivita_robot.py
python3 test/test_ros2_driver.py
python3 test/test_camera_orbbec.py
python3 test/test_mujoco.py
python3 test/test_ai_components.py
python3 test/test_web_interface.py
```

## Checklist Componenti

### ✅ Base
- [ ] Ubuntu 22.04
- [ ] Python 3.10+
- [ ] ROS2 Humble
- [ ] Docker (opzionale)

### ✅ Driver UR
- [ ] UR ROS2 Driver
- [ ] ur_rtde
- [ ] Workspace ROS2

### ✅ MuJoCo
- [ ] MuJoCo installato
- [ ] MuJoCo Menagerie
- [ ] Modelli UR

### ✅ Camera
- [ ] OrbbecSDK ROS2
- [ ] Topics camera

### ✅ AI
- [ ] YOLOv8
- [ ] OpenCV
- [ ] Open3D

### ✅ Planning
- [ ] MoveIt2

### ✅ Custom
- [ ] Web Interface
- [ ] ROS2 Bridge

## Prerequisiti Test

### ROS2 Driver
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194
```

### Camera
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

### ROS2 Daemon
```bash
ros2 daemon start
```

## Interpretazione

- ✅ Verde = OK
- ❌ Rosso = Mancante/Richiesto
- ⚠️ Giallo = Opzionale/Da verificare

## File Risultati

- `~/test_sistema_results.json` - Risultati completi

## Documentazione

- `GUIDA_TEST_COMPLETA.md` - Guida dettagliata
- `RIEPILOGO_TEST_SISTEMA.md` - Riepilogo completo





