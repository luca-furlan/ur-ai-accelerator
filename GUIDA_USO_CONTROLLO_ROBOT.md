# 🎮 GUIDA COMPLETA: COSA PUOI USARE E COME

## 📋 INDICE

1. [Script Disponibili](#script-disponibili)
2. [Come Usare il Controllo Robot](#come-usare-il-controllo-robot)
3. [Troubleshooting Completo](#troubleshooting-completo)
4. [Verifica Log](#verifica-log)
5. [Problemi Comuni](#problemi-comuni)

---

## 📁 SCRIPT DISPONIBILI

### Script di Controllo Robot

| Script | Descrizione | Quando Usare |
|--------|-------------|--------------|
| `controllo_robot_senza_urcap.py` | Controllo base senza URCap | ✅ **INIZIA QUI** - Funziona subito |
| `controllo_mujoco_robot.py` | Controllo con simulazione MuJoCo | Per testare traiettorie prima |
| `controllo_joystick_fisico.py` | Controllo con joystick USB | Se hai joystick fisico |
| `remote_ur_control/web_interface.py` | Web interface browser | Controllo da browser |

### Script di Verifica e Test

| Script | Descrizione | Quando Usare |
|--------|-------------|--------------|
| `troubleshoot_robot_completo.py` | **TROUBLESHOOTING COMPLETO** | ⚠️ **USA QUESTO** se qualcosa non funziona |
| `verifica_e_testa_controllo.py` | Test interattivo controllo | Per verificare funzionamento |
| `check_robot_real_status.py` | Verifica stato robot | Per vedere stato generale |
| `test_rtde_robot.py` | Test connessione RTDE | Per testare RTDE |

### Script di Avvio Sistema

| Script | Descrizione |
|--------|-------------|
| `avvia_controllo_robot.sh` | Menu interattivo per scegliere metodo |
| `avvia_web_interface_ur5e.sh` | Avvia web interface |
| `avvia_driver_ur5e.sh` | Avvia driver ROS2 (richiede URCap) |

---

## 🚀 COME USARE IL CONTROLLO ROBOT

### Metodo 1: Controllo Base (SENZA URCap) - ✅ RACCOMANDATO PER INIZIARE

**Funziona SUBITO, non richiede installazione URCap!**

```bash
# Connettiti all'AI Accelerator
ssh lab@192.168.10.191

# Vai nella directory
cd ~/MekoAiAccelerator

# Esegui script
python3 controllo_robot_senza_urcap.py
```

**Cosa puoi fare:**
- ✅ Leggere posizioni joints
- ✅ Inviare comandi movimento (limitati)
- ✅ Testare connessione robot

**Limitazioni:**
- Movimento meno fluido
- Alcune funzioni avanzate non disponibili

---

### Metodo 2: Web Interface (Browser) - ✅ PIÙ FACILE

**Controllo da browser con joystick virtuale!**

```bash
# Connettiti all'AI Accelerator
ssh lab@192.168.10.191

# Vai nella directory
cd ~/MekoAiAccelerator

# Avvia web interface
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

**Poi apri browser:**
```
http://192.168.10.191:8081
```

**Cosa puoi fare:**
- ✅ Controllo robot con joystick virtuale
- ✅ Movimento fluido
- ✅ Interfaccia grafica moderna
- ✅ Controllo da remoto

---

### Metodo 3: MuJoCo (Simulazione + Reale)

**Testa traiettorie in simulazione prima di eseguirle!**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 controllo_mujoco_robot.py
```

**Cosa puoi fare:**
- ✅ Visualizza robot 3D
- ✅ Muovi in simulazione
- ✅ Invia comandi al robot reale

---

### Metodo 4: Driver ROS2 (CONTROLLO COMPLETO)

**Richiede installazione External Control URCap**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Avvia driver
bash avvia_driver_ur5e.sh
```

**Sul Teach Pendant:**
1. Avvia programma con **External Control**
2. IP: `192.168.10.191`, Porta: `50002`
3. Premi **PLAY**

---

## 🔧 TROUBLESHOOTING COMPLETO

### Se il Controllo Robot NON Funziona

**1. Esegui troubleshooting completo:**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 troubleshoot_robot_completo.py
```

Questo script:
- ✅ Testa connettività di rete
- ✅ Verifica tutte le porte robot
- ✅ Testa Primary Interface
- ✅ Testa RTDE
- ✅ Testa Dashboard Server
- ✅ Testa Remote Controller
- ✅ **Salva log dettagliato** in `~/MekoAiAccelerator/logs/`

**2. Verifica log:**

```bash
# Vedi ultimo log
ls -lt ~/MekoAiAccelerator/logs/ | head -5

# Leggi log
cat ~/MekoAiAccelerator/logs/troubleshoot_*.log | tail -100
```

---

### Test Interattivo

**Per testare il controllo passo-passo:**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 verifica_e_testa_controllo.py
```

Menu interattivo:
1. Test controllo senza URCap
2. Test lettura RTDE
3. Test Remote Controller
4. Test movimento sicuro (5mm)
5. Esegui tutti i test

---

## 📊 VERIFICA LOG

### Dove sono i log?

Tutti i log sono salvati in:
```
~/MekoAiAccelerator/logs/
```

### Come vedere i log

```bash
# Lista tutti i log
ls -lh ~/MekoAiAccelerator/logs/

# Vedi ultimo log troubleshooting
tail -f ~/MekoAiAccelerator/logs/troubleshoot_*.log

# Cerca errori
grep -i error ~/MekoAiAccelerator/logs/*.log

# Cerca problemi specifici
grep -i "timeout\|connection\|refused" ~/MekoAiAccelerator/logs/*.log
```

### Script per analisi log

```bash
# Crea script analisi
cat > ~/MekoAiAccelerator/analizza_log.sh << 'EOF'
#!/bin/bash
LOG_DIR="$HOME/MekoAiAccelerator/logs"
echo "=== ULTIMI ERRORI ==="
grep -i error "$LOG_DIR"/*.log | tail -20
echo ""
echo "=== ULTIMI WARNING ==="
grep -i warning "$LOG_DIR"/*.log | tail -20
echo ""
echo "=== TEST FALLITI ==="
grep -i "❌\|FAIL" "$LOG_DIR"/*.log | tail -20
EOF

chmod +x ~/MekoAiAccelerator/analizza_log.sh
```

---

## ⚠️ PROBLEMI COMUNI

### 1. "Connection refused" o "Timeout"

**Causa:** Robot non raggiungibile o non in RUNNING

**Soluzione:**
```bash
# Verifica connettività
ping 192.168.10.194

# Verifica che robot sia in RUNNING
# (Controlla Teach Pendant)
```

---

### 2. "ModuleNotFoundError: No module named 'ur_rtde'"

**Causa:** Libreria ur_rtde non installata

**Soluzione:**
```bash
pip3 install --user ur-rtde
# O
python3 -m pip install --user ur-rtde
```

---

### 3. "Robot non si muove"

**Causa possibili:**
- Robot in modalità PROTECTIVE STOP
- Nessun programma in esecuzione
- Emergency stop attivo

**Soluzione:**
1. Controlla Teach Pendant
2. Verifica che robot sia in RUNNING
3. Se usi External Control, verifica che programma sia avviato

---

### 4. "Port 30001/30002/30004 non raggiungibile"

**Causa:** Porte chiuse o robot non in RUNNING

**Soluzione:**
```bash
# Esegui troubleshooting completo
python3 troubleshoot_robot_completo.py

# Verifica stato robot
python3 check_robot_real_status.py
```

---

### 5. "Web Interface non si connette"

**Causa:** Porta occupata o firewall

**Soluzione:**
```bash
# Verifica porta
netstat -tuln | grep 8081

# Prova porta diversa
export WEB_PORT=8082
python3 -m remote_ur_control.web_interface
```

---

## 📋 CHECKLIST PRE-UTILIZZO

Prima di usare il controllo robot, verifica:

- [ ] Robot acceso e raggiungibile (`ping 192.168.10.194`)
- [ ] Robot in modalità RUNNING (controlla Teach Pendant)
- [ ] Nessun emergency stop attivo
- [ ] Connessione di rete OK
- [ ] Librerie Python installate (`ur_rtde`, etc.)

**Per verifica rapida:**
```bash
python3 troubleshoot_robot_completo.py
```

---

## 🎯 RACCOMANDAZIONE

**Per iniziare SUBITO:**

1. **Esegui troubleshooting:**
   ```bash
   python3 troubleshoot_robot_completo.py
   ```

2. **Se tutto OK, prova controllo base:**
   ```bash
   python3 controllo_robot_senza_urcap.py
   ```

3. **O usa web interface:**
   ```bash
   python3 -m remote_ur_control.web_interface
   ```

4. **Se problemi, controlla log:**
   ```bash
   tail -f ~/MekoAiAccelerator/logs/*.log
   ```

---

## 📞 SUPPORTO

Se hai problemi:

1. **Esegui troubleshooting completo** e salva log
2. **Controlla log** per errori specifici
3. **Verifica stato robot** sul Teach Pendant
4. **Controlla questa guida** per problemi comuni

**File di riferimento:**
- `troubleshoot_robot_completo.py` - Troubleshooting completo
- `verifica_e_testa_controllo.py` - Test interattivo
- `~/MekoAiAccelerator/logs/` - Tutti i log

---

**Ultimo aggiornamento:** $(date)




