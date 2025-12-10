# 🌐 GUIDA: Web Interface con Stato Robot

## ✅ COSA È STATO AGGIUNTO

La web interface ora mostra **lo stato completo del robot in tempo reale**:

1. **Robot Status Panel** - Nuovo pannello che mostra:
   - Robot Mode (RUNNING, IDLE, POWER_OFF, etc.)
   - Safety Mode (NORMAL, PROTECTIVE_STOP, etc.)
   - Program State (PLAYING, STOPPED, etc.)
   - Remote Control (true/false)
   - **Joint Positions** (6 valori in rad)
   - **TCP Pose** (x, y, z, rx, ry, rz)

2. **Aggiornamento Automatico** - Lo stato si aggiorna ogni 2 secondi

3. **API Endpoint** - Nuovo endpoint `/api/robot_status` per leggere lo stato

---

## 🚀 COME USARE

### 1. Avvia Web Interface

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator

# Configura
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8081

# Avvia
python3 -m remote_ur_control.web_interface
```

Dovresti vedere:
```
✅ ROS2 bridge initialized - publishing at 125Hz
🌐 Starting web interface on http://0.0.0.0:8081
```

### 2. Apri Browser

Apri il browser su:
```
http://192.168.10.191:8081
```

### 3. Vedi Stato Robot

Nella pagina vedrai:

**In alto:**
- **ROS2 Monitor** - Stato ROS2 bridge
- **Robot Status** - **NUOVO!** Stato robot in tempo reale

**Robot Status mostra:**
- Robot Mode: `RUNNING` (verde) o altro (rosso)
- Safety Mode: `NORMAL` (verde) o altro (giallo)
- Program State: `PLAYING` (verde) o altro (rosso)
- Remote Control: `true` (verde) o `false` (rosso)
- Joint Positions: `[0.1234, -1.5678, ...]` (6 valori)
- TCP Pose: `[0.1234, 0.5678, 0.9012, ...]` (6 valori)

**Aggiornamento:**
- Automatico ogni 2 secondi
- Clicca "Aggiorna stato robot" per aggiornare manualmente

---

## 🧪 TEST

### Test Web Interface

```bash
# Dalla tua macchina (non dall'AI Accelerator)
python3 testa_web_interface.py
```

Questo script:
- ✅ Verifica che la web interface sia raggiungibile
- ✅ Testa endpoint `/api/status` (ROS2)
- ✅ Testa endpoint `/api/robot_status` (Robot) - **NUOVO!**
- ✅ Mostra lo stato del robot

### Test Manuale API

```bash
# Test robot status API
curl http://192.168.10.191:8081/api/robot_status | python3 -m json.tool
```

Dovresti vedere:
```json
{
  "status": "ok",
  "data": {
    "dashboard": {
      "robotmode": "RUNNING",
      "safetymode": "NORMAL",
      "programState": "PLAYING",
      "remote_control": "true"
    },
    "rtde": {
      "joints": [0.1234, -1.5678, 1.2345, -0.5678, 1.5678, 0.1234],
      "tcp_pose": [0.1234, 0.5678, 0.9012, 0.0, 3.1415, 0.0]
    }
  }
}
```

---

## ⚠️ TROUBLESHOOTING

### Web Interface non si connette

**Problema:** `Connection refused` o timeout

**Soluzione:**
1. Verifica che la web interface sia avviata:
   ```bash
   # Sull'AI Accelerator
   ps aux | grep web_interface
   ```

2. Verifica porta:
   ```bash
   netstat -tuln | grep 8081
   ```

3. Riavvia web interface:
   ```bash
   export UR_ROBOT_IP=192.168.10.194
   export WEB_PORT=8081
   python3 -m remote_ur_control.web_interface
   ```

---

### Robot Status mostra "unknown" o errori

**Problema:** Dashboard o RTDE non raggiungibili

**Soluzione:**

1. **Dashboard Error:**
   - Verifica che robot sia acceso
   - Verifica connettività: `ping 192.168.10.194`
   - Verifica porta 29999: `telnet 192.168.10.194 29999`

2. **RTDE Error:**
   - Verifica che `ur_rtde` sia installato:
     ```bash
     python3 -c "import ur_rtde; print('OK')"
     ```
   - Se non installato:
     ```bash
     pip3 install --user ur-rtde
     ```
   - Verifica porta 30004: `telnet 192.168.10.194 30004`

---

### Joint Positions o TCP Pose non si aggiornano

**Problema:** RTDE non riceve dati

**Soluzione:**
1. Verifica che robot sia in RUNNING
2. Verifica che programma sia in PLAYING
3. Controlla log web interface per errori RTDE

---

## 📊 COSA VEDERE NELL'INTERFACCIA

### Stato Normale (Robot Pronto)

```
Robot Mode: RUNNING (verde)
Safety Mode: NORMAL (verde)
Program State: PLAYING (verde)
Remote Control: true (verde)
Joint Positions: [valori aggiornati ogni 2s]
TCP Pose: [valori aggiornati ogni 2s]
```

### Stato Problema

```
Robot Mode: POWER_OFF (rosso)  ← Robot spento
Safety Mode: PROTECTIVE_STOP (giallo)  ← Stop di sicurezza
Program State: STOPPED (rosso)  ← Programma non in esecuzione
Remote Control: false (rosso)  ← Remote control non attivo
```

---

## 🔧 API ENDPOINTS

### GET /api/robot_status

Legge lo stato completo del robot.

**Response:**
```json
{
  "status": "ok",
  "data": {
    "dashboard": {
      "robotmode": "RUNNING",
      "safetymode": "NORMAL",
      "programState": "PLAYING",
      "remote_control": "true"
    },
    "rtde": {
      "joints": [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
      "tcp_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "tcp_speed": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      "robot_status": 12345
    }
  }
}
```

**Errori possibili:**
- `dashboard.error`: Errore connessione Dashboard Server
- `rtde.error`: Errore connessione RTDE (es. "ur_rtde not installed")

---

## ✅ CHECKLIST

Prima di usare la web interface:

- [ ] Robot acceso e raggiungibile (`ping 192.168.10.194`)
- [ ] Robot in modalità RUNNING
- [ ] Programma in esecuzione (PLAYING)
- [ ] Remote control attivo
- [ ] Web interface avviata
- [ ] Browser può raggiungere `http://192.168.10.191:8081`

---

## 🎯 PROSSIMI PASSI

1. **Avvia web interface:**
   ```bash
   python3 -m remote_ur_control.web_interface
   ```

2. **Apri browser:**
   ```
   http://192.168.10.191:8081
   ```

3. **Verifica stato robot:**
   - Controlla pannello "Robot Status"
   - Dovresti vedere joints e TCP pose aggiornarsi ogni 2 secondi

4. **Se tutto OK:**
   - Usa joystick per controllare robot
   - Vedi stato aggiornarsi in tempo reale

---

**Ultimo aggiornamento:** $(date)

**Per problemi:** Controlla log web interface o esegui `python3 testa_web_interface.py`




