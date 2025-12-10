# 🚀 Istruzioni Avvio Test Sistema

## 📍 Dove Sono i File?

I file di test sono attualmente sulla **macchina locale (Windows)** in:
- `C:\Users\user\Documents\MekoAiAccelerator\`
- Directory `test/` con tutti i test

## 🔄 Trasferimento su AI Accelerator

### Opzione 1: Script Automatico (Linux/Mac)

Se hai accesso a Linux/Mac o WSL:

```bash
# 1. Trasferisci i file
./deploy_test_sistema.sh

# 2. Avvia test
./avvia_test_sistema.sh quick          # Quick check
./avvia_test_sistema.sh completo      # Verifica completa
./avvia_test_sistema.sh tutti         # Tutti i test
```

### Opzione 2: Trasferimento Manuale

#### Da Windows (PowerShell o CMD):

```powershell
# 1. Connettiti via SSH
ssh lab@192.168.10.191
# Password: easybot

# 2. Crea directory se non esiste
mkdir -p ~/MekoAiAccelerator/test

# 3. Da Windows, trasferisci file con SCP (se disponibile)
# Oppure usa WinSCP, FileZilla, o altro client SFTP
```

#### Con WinSCP o FileZilla:
- **Host:** 192.168.10.191
- **User:** lab
- **Password:** easybot
- **Porta:** 22

Trasferisci:
- `test_sistema_completo.py`
- `quick_check_sistema.py`
- Tutta la directory `test/`
- File di documentazione (opzionale)

### Opzione 3: Copia/Incolla (VNC)

1. Connettiti via VNC: `192.168.10.191:5901`
2. Apri terminale
3. Crea file manualmente o copia/incolla contenuto

## ▶️ Avvio Test sulla Macchina AI Accelerator

### Via SSH:

```bash
# 1. Connettiti
ssh lab@192.168.10.191
# Password: easybot

# 2. Vai nella directory
cd ~/MekoAiAccelerator

# 3. Rendi eseguibili (se necessario)
chmod +x test_sistema_completo.py quick_check_sistema.py
chmod +x test/*.py

# 4. Avvia test
```

### Test Disponibili:

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

## 🎯 Quick Start

### 1. Trasferisci File (una volta)

```bash
# Da Windows, usa WinSCP o:
# - Connettiti via SSH
# - Crea file manualmente copiando contenuto
```

### 2. Avvia Quick Check

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 quick_check_sistema.py
```

### 3. Verifica Completa

```bash
python3 test_sistema_completo.py
```

## 📋 File da Trasferire

**File principali:**
- `test_sistema_completo.py`
- `quick_check_sistema.py`

**Directory test/ (tutti i file):**
- `test/run_all_tests.py`
- `test/test_connettivita_robot.py`
- `test/test_ros2_driver.py`
- `test/test_camera_orbbec.py`
- `test/test_mujoco.py`
- `test/test_ai_components.py`
- `test/test_web_interface.py`
- `test/__init__.py`
- `test/README.md`

**Documentazione (opzionale):**
- `GUIDA_TEST_COMPLETA.md`
- `RIEPILOGO_TEST_SISTEMA.md`
- `QUICK_REFERENCE_TEST.md`

## 🔧 Prerequisiti sulla Macchina AI Accelerator

I test richiedono:
- Python 3.10+
- ROS2 Humble (per alcuni test)
- Pacchetti Python installati (ur_rtde, mujoco, opencv, etc.)

## 💡 Suggerimento

Se hai già i file sulla macchina AI Accelerator (da deploy precedenti), potrebbero già essere presenti. Verifica:

```bash
ssh lab@192.168.10.191
ls -la ~/MekoAiAccelerator/test_sistema_completo.py
ls -la ~/MekoAiAccelerator/test/
```

Se esistono, puoi avviarli direttamente!

---

**Nota:** I file sono sulla macchina locale Windows. Devi trasferirli sulla macchina AI Accelerator (Linux) per eseguirli.





