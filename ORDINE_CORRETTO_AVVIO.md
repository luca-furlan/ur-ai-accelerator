# 🚀 ORDINE CORRETTO DI AVVIO - RISOLVE "CONNECTION REFUSED"

## 🔴 PROBLEMA

Quando provi ad attivare Remote Control sul Teach Pendant, vedi:
```
192.168.10.191:50002 connection refused
```

**Causa:** Il driver ROS2 non è ancora avviato, quindi la porta 50002 non è aperta!

---

## ✅ SOLUZIONE: ORDINE CORRETTO

Il driver ROS2 **DEVE essere avviato PRIMA** di attivare Remote Control sul Teach Pendant!

### 📋 PROCEDURA CORRETTA

#### Passo 1: Avvia Driver ROS2 (Terminale SSH)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x avvia_driver_ros2.sh
./avvia_driver_ros2.sh
```

**LASCIA QUESTO TERMINALE APERTO!** Il driver deve rimanere attivo.

**Aspetta fino a vedere:**
```
[INFO] System successfully started!
```

Questo significa che la porta 50002 è aperta e pronta ad accettare connessioni.

---

#### Passo 2: Sul Teach Pendant

**SOLO DOPO che il driver ROS2 è avviato:**

1. Vai su **Program**
2. Apri o crea programma con nodo **External Control**
3. Configura **External Control**:
   - **IP Host:** `192.168.10.191`
   - **Porta:** `50002`
   - **Host Name:** (vuoto)
4. **SALVA** il programma
5. **Attiva Remote Control** sul Teach Pendant
6. **Premi PLAY** sul programma External Control
7. Verifica che:
   - Programma sia in stato **PLAYING**
   - Remote Control sia **ATTIVO**
   - Nessun errore "connection refused"

---

#### Passo 3: Avvia Web Interface (Nuovo Terminale SSH)

**In un NUOVO terminale:**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

---

#### Passo 4: Dalla Web Interface

1. Apri `http://192.168.10.191:8080`
2. Verifica nel pannello "Robot Status" che:
   - **Robot Mode:** RUNNING
   - **Remote Control:** true ✅
   - **Program State:** PLAYING
3. Nel pannello "Sistema Robot", verifica che:
   - **Driver ROS2:** ✅ Attivo
   - **Porta 50002:** ✅ Aperta
4. Clicca **"🔄 Switch Controller"** per attivare il controller
5. Usa i joystick per muovere il robot!

---

## ⚠️ ERRORE COMUNE

### ❌ SBAGLIATO:
1. Attivi Remote Control sul Teach Pendant
2. Poi avvii il driver ROS2
3. **Risultato:** "connection refused" perché la porta 50002 non era aperta

### ✅ CORRETTO:
1. Avvii il driver ROS2 PRIMA
2. Aspetti che la porta 50002 sia aperta
3. Poi attivi Remote Control sul Teach Pendant
4. **Risultato:** Connessione riuscita!

---

## 🔍 VERIFICA CHE FUNZIONI

### Verifica Porta 50002 Aperta:
```bash
netstat -tuln | grep 50002
```

**Dovresti vedere:**
```
tcp  0  0  0.0.0.0:50002  0.0.0.0:*  LISTEN
```

### Verifica Driver ROS2 Attivo:
```bash
ps aux | grep ur_ros2_control_node
```

**Dovresti vedere il processo attivo.**

### Verifica Connessione Robot:
Nel log del driver ROS2 dovresti vedere:
```
[INFO] Connected: Universal Robots External Control
```

---

## 🛑 FERMARE TUTTO

### Ferma Driver ROS2:
Nel terminale dove è avviato:
- Premi **CTRL+C**

OPPURE:
```bash
pkill -f ur_ros2_control_node
```

### Ferma Web Interface:
Nel terminale dove è avviata:
- Premi **CTRL+C**

OPPURE:
```bash
pkill -f web_interface
```

---

## ✅ RIEPILOGO

**Ordine corretto:**
1. ✅ Avvia driver ROS2 (porta 50002 si apre)
2. ✅ Attiva Remote Control sul Teach Pendant
3. ✅ Avvia programma External Control sul Teach Pendant
4. ✅ Avvia web interface
5. ✅ Switch controller dalla web interface
6. ✅ Usa joystick!

**NON funziona se:**
- ❌ Attivi Remote Control prima di avviare il driver ROS2
- ❌ Il driver ROS2 non è attivo quando il robot cerca di connettersi

