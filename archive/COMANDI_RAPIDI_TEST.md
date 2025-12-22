# ⚡ Comandi Rapidi - Test Sistema

## 📍 Dove Sono i File?

**Sulla macchina locale (Windows):**
- `C:\Users\user\Documents\MekoAiAccelerator\test_sistema_completo.py`
- `C:\Users\user\Documents\MekoAiAccelerator\test\` (tutti i test)

**Sulla macchina AI Accelerator (dopo trasferimento):**
- `~/MekoAiAccelerator/test_sistema_completo.py`
- `~/MekoAiAccelerator/test/` (tutti i test)

## 🚀 Avvio Rapido

### Opzione 1: Via SSH (Diretto)

```bash
# 1. Connettiti alla macchina AI Accelerator
ssh lab@192.168.10.191
# Password: easybot

# 2. Vai nella directory
cd ~/MekoAiAccelerator

# 3. Avvia test
python3 quick_check_sistema.py          # Quick check
python3 test_sistema_completo.py        # Verifica completa
python3 test/run_all_tests.py           # Tutti i test
```

### Opzione 2: Da Windows (PowerShell)

```powershell
# Avvia test direttamente via SSH
.\avvia_test_sistema.ps1 quick          # Quick check
.\avvia_test_sistema.ps1 completo      # Verifica completa
.\avvia_test_sistema.ps1 tutti         # Tutti i test
```

### Opzione 3: Script Bash (Linux/Mac/WSL)

```bash
./avvia_test_sistema.sh quick
./avvia_test_sistema.sh completo
./avvia_test_sistema.sh tutti
```

## 📋 Test Disponibili

### Quick Check (Rapido)
```bash
python3 quick_check_sistema.py
```
Verifica componenti principali in pochi secondi.

### Verifica Completa
```bash
python3 test_sistema_completo.py
```
Verifica completa di tutti i componenti installati.

### Tutti i Test Funzionali
```bash
python3 test/run_all_tests.py
```
Esegue tutti i test funzionali in sequenza.

### Test Singoli
```bash
python3 test/test_connettivita_robot.py    # Test robot
python3 test/test_ros2_driver.py          # Test ROS2
python3 test/test_camera_orbbec.py        # Test camera
python3 test/test_mujoco.py               # Test MuJoCo
python3 test/test_ai_components.py       # Test AI
python3 test/test_web_interface.py        # Test web
```

## 🔄 Trasferimento File (Se Necessario)

### Con WinSCP/FileZilla
1. Apri WinSCP o FileZilla
2. Connetti a: `192.168.10.191` (user: `lab`, password: `easybot`)
3. Trasferisci:
   - `test_sistema_completo.py`
   - `quick_check_sistema.py`
   - Directory `test/` completa

### Con SCP (da terminale)
```bash
# Da Linux/Mac/WSL
scp test_sistema_completo.py lab@192.168.10.191:~/MekoAiAccelerator/
scp -r test/ lab@192.168.10.191:~/MekoAiAccelerator/
```

## ✅ Verifica File Presenti

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
ls -la test_sistema_completo.py
ls -la test/
```

Se i file sono presenti, puoi avviarli direttamente!

## 🎯 Quick Start Completo

```bash
# 1. Connettiti
ssh lab@192.168.10.191

# 2. Vai nella directory
cd ~/MekoAiAccelerator

# 3. Quick check
python3 quick_check_sistema.py

# 4. Se tutto OK, verifica completa
python3 test_sistema_completo.py
```

## 📊 Output

- ✅ **Verde** = Componente OK
- ❌ **Rosso** = Componente mancante/errore
- ⚠️ **Giallo** = Warning/opzionale

## 📁 File Risultati

Dopo `test_sistema_completo.py`:
- `~/test_sistema_results.json` - Risultati in formato JSON

---

**Nota:** I file devono essere sulla macchina AI Accelerator per essere eseguiti. Se non ci sono, trasferiscili prima!











