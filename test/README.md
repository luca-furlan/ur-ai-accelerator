# Test Suite - AI Accelerator

Questa directory contiene tutti i test per verificare il funzionamento del sistema.

## Quick Start

```bash
# Esegui tutti i test
python3 run_all_tests.py

# Test specifico
python3 test_connettivita_robot.py
python3 test_ros2_driver.py
python3 test_camera_orbbec.py
python3 test_mujoco.py
python3 test_ai_components.py
python3 test_web_interface.py
```

## File Test

- `run_all_tests.py` - Esegue tutti i test in sequenza
- `test_connettivita_robot.py` - Test connessione robot UR
- `test_ros2_driver.py` - Test ROS2 driver UR
- `test_camera_orbbec.py` - Test camera Orbbec
- `test_mujoco.py` - Test MuJoCo
- `test_ai_components.py` - Test componenti AI (YOLO, OpenCV, Open3D)
- `test_web_interface.py` - Test web interface

## Verifica Completa Sistema

Per una verifica completa di tutti i componenti installati, eseguire dalla root del progetto:

```bash
cd ~/MekoAiAccelerator
python3 test_sistema_completo.py
```

Vedi `../GUIDA_TEST_COMPLETA.md` per documentazione dettagliata.





