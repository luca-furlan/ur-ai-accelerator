# 🚀 COSA PUOI USARE ADESSO - GUIDA PRATICA

## ✅ STATO ATTUALE

**Installazione base: 100% completata** ✅  
**Controllo robot: Da verificare e testare** ⚠️

---

## 🎯 COSA PUOI FARE SUBITO

### 1. 🔧 TROUBLESHOOTING COMPLETO (PRIMA COSA DA FARE!)

**Se il controllo robot non ha funzionato finora, esegui questo:**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 troubleshoot_robot_completo.py
```

**Cosa fa:**
- ✅ Testa connettività di rete
- ✅ Verifica tutte le porte robot (30001, 30002, 30004, 29999)
- ✅ Testa Primary Interface (invio comandi)
- ✅ Testa RTDE (lettura dati)
- ✅ Testa Dashboard Server
- ✅ Testa Remote Controller
- ✅ **Salva log dettagliato** per analisi

**Dove sono i log:**
```
~/MekoAiAccelerator/logs/troubleshoot_YYYYMMDD_HHMMSS.log
```

---

### 2. 📊 VERIFICA E TEST INTERATTIVO

**Per testare il controllo passo-passo:**

```bash
python3 verifica_e_testa_controllo.py
```

**Menu disponibile:**
1. Test controllo senza URCap (Primary Interface)
2. Test lettura RTDE
3. Test Remote Controller
4. Test movimento sicuro (5mm) - Richiede conferma
5. Esegui tutti i test

---

### 3. 📋 VISUALIZZA LOG

**Per vedere i log in modo esaustivo:**

```bash
python3 visualizza_log.py
```

**Opzioni:**
- Riepilogo ultimo log (errori, warning, successi)
- Ultime 50/100 righe
- Risultati test
- Ricerca pattern in tutti i log

**O manualmente:**
```bash
# Vedi ultimo log
tail -f ~/MekoAiAccelerator/logs/troubleshoot_*.log

# Cerca errori
grep -i error ~/MekoAiAccelerator/logs/*.log

# Cerca problemi
grep -i "timeout\|connection\|refused" ~/MekoAiAccelerator/logs/*.log
```

---

### 4. 🤖 CONTROLLO ROBOT BASE

**Se troubleshooting OK, prova controllo:**

```bash
python3 controllo_robot_senza_urcap.py
```

**Cosa puoi fare:**
- Leggere posizioni joints
- Inviare comandi movimento (limitati)
- Testare connessione

---

### 5. 🌐 WEB INTERFACE

**Controllo da browser (più facile):**

```bash
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081
python3 -m remote_ur_control.web_interface
```

**Poi apri browser:**
```
http://192.168.10.191:8081
```

**Cosa puoi fare:**
- Controllo robot con joystick virtuale
- Movimento fluido
- Interfaccia grafica moderna

---

### 6. 🎮 MUJOCO SIMULAZIONE

**Visualizza e simula robot:**

```bash
python3 -m mujoco.viewer --mjcf ~/mujoco_menagerie/universal_robots_ur5e/scene.xml
```

**O controllo con simulazione:**
```bash
python3 controllo_mujoco_robot.py
```

---

### 7. 📡 VERIFICA STATO ROBOT

**Verifica stato generale:**

```bash
python3 check_robot_real_status.py
```

**O test RTDE:**
```bash
python3 test_rtde_robot.py
```

---

## 📁 SCRIPT DISPONIBILI

### Troubleshooting e Diagnostica
- ✅ `troubleshoot_robot_completo.py` - **TROUBLESHOOTING COMPLETO**
- ✅ `verifica_e_testa_controllo.py` - Test interattivo
- ✅ `visualizza_log.py` - Visualizza log
- ✅ `check_robot_real_status.py` - Verifica stato robot
- ✅ `test_rtde_robot.py` - Test RTDE

### Controllo Robot
- ✅ `controllo_robot_senza_urcap.py` - Controllo base
- ✅ `controllo_mujoco_robot.py` - Controllo con MuJoCo
- ✅ `controllo_joystick_fisico.py` - Controllo joystick
- ✅ `remote_ur_control/web_interface.py` - Web interface

### Avvio Sistema
- ✅ `avvia_controllo_robot.sh` - Menu interattivo
- ✅ `avvia_web_interface_ur5e.sh` - Avvia web interface
- ✅ `avvia_driver_ur5e.sh` - Avvia driver ROS2

---

## 🔍 WORKFLOW RACCOMANDATO

### Se il Controllo NON Funziona:

1. **Esegui troubleshooting completo:**
   ```bash
   python3 troubleshoot_robot_completo.py
   ```

2. **Visualizza log:**
   ```bash
   python3 visualizza_log.py
   # O
   tail -f ~/MekoAiAccelerator/logs/troubleshoot_*.log
   ```

3. **Analizza errori:**
   - Cerca "❌" o "ERROR" nei log
   - Verifica quale test è fallito
   - Controlla messaggi di errore specifici

4. **Risolvi problemi:**
   - Se "Connection refused" → Robot non in RUNNING
   - Se "Timeout" → Problema di rete
   - Se "ModuleNotFoundError" → Installa librerie mancanti

5. **Riprova:**
   ```bash
   python3 verifica_e_testa_controllo.py
   ```

---

### Se il Controllo Funziona:

1. **Prova controllo base:**
   ```bash
   python3 controllo_robot_senza_urcap.py
   ```

2. **O usa web interface:**
   ```bash
   python3 -m remote_ur_control.web_interface
   ```

3. **Testa movimento sicuro:**
   ```bash
   python3 verifica_e_testa_controllo.py
   # Scegli opzione 4 (movimento sicuro)
   ```

---

## 📊 LOG E DIAGNOSTICA

### Dove sono i log?

```
~/MekoAiAccelerator/logs/
```

### File di log generati:

- `troubleshoot_YYYYMMDD_HHMMSS.log` - Log troubleshooting completo
- `test_controllo_YYYYMMDD_HHMMSS.log` - Log test controllo

### Come analizzare log:

```bash
# Script automatico
python3 visualizza_log.py

# Manuale
tail -100 ~/MekoAiAccelerator/logs/troubleshoot_*.log | grep -i error
```

---

## ⚠️ PROBLEMI COMUNI E SOLUZIONI

### 1. "Connection refused" o "Timeout"

**Causa:** Robot non raggiungibile o non in RUNNING

**Soluzione:**
- Verifica che robot sia acceso
- Verifica che robot sia in modalità RUNNING (Teach Pendant)
- Verifica connettività: `ping 192.168.10.194`

---

### 2. "ModuleNotFoundError: No module named 'ur_rtde'"

**Soluzione:**
```bash
pip3 install --user ur-rtde
```

---

### 3. "Robot non si muove"

**Causa possibili:**
- Robot in PROTECTIVE STOP
- Emergency stop attivo
- Nessun programma in esecuzione

**Soluzione:**
- Controlla Teach Pendant
- Verifica che robot sia in RUNNING
- Se usi External Control, verifica programma avviato

---

### 4. "Port non raggiungibile"

**Soluzione:**
```bash
# Esegui troubleshooting completo
python3 troubleshoot_robot_completo.py

# Verifica quale porta fallisce
# Controlla log per dettagli
```

---

## ✅ CHECKLIST PRE-UTILIZZO

Prima di usare il controllo robot:

- [ ] Robot acceso e raggiungibile (`ping 192.168.10.194`)
- [ ] Robot in modalità RUNNING (controlla Teach Pendant)
- [ ] Nessun emergency stop attivo
- [ ] Connessione di rete OK
- [ ] Librerie Python installate

**Verifica rapida:**
```bash
python3 troubleshoot_robot_completo.py
```

---

## 📚 DOCUMENTAZIONE

- `GUIDA_USO_CONTROLLO_ROBOT.md` - Guida completa dettagliata
- `STATO_INSTALLAZIONE_AGGIORNATO.md` - Stato installazione
- `RIEPILOGO_STATO_ATTUALE.md` - Riepilogo stato

---

## 🎯 PROSSIMI PASSI

1. **Esegui troubleshooting completo** per capire cosa non funziona
2. **Analizza log** per vedere errori specifici
3. **Risolvi problemi** trovati
4. **Testa controllo** con script interattivo
5. **Usa controllo** per muovere robot

---

**Ultimo aggiornamento:** $(date)

**Per supporto:** Controlla log in `~/MekoAiAccelerator/logs/`










