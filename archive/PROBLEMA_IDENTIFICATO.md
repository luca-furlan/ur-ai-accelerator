# 🚨 PROBLEMA IDENTIFICATO - CRASH DRIVER ROS2

## 📊 Stato Attuale

✅ **Programma in PLAYING**: `remote_control.urp`  
✅ **Robot in RUNNING mode**  
✅ **RTDE disponibile** (porta 30004 aperta)  
❌ **Porta 50002 CHIUSA** (External Control NON attivo)

## 🔴 CAUSA DEL CRASH

Il driver ROS2 si connette alla **porta 50002** per il controllo tramite External Control URCap.  
Se questa porta è chiusa, il driver **va in crash** perché non riesce a stabilire la connessione.

## ✅ SOLUZIONE

### Configura External Control sul Teach Pendant

1. **Vai sul TEACH PENDANT del robot**

2. **Apri il programma `remote_control.urp`**

3. **Verifica/Aggiungi nodo "External Control"**:
   - Il nodo deve essere presente nel programma
   - Se non c'è:
     - Vai su: **Installation** → **URCaps**
     - Verifica che **"External Control"** sia installato
     - Se non installato, installalo
     - Torna al programma e aggiungi il nodo **"External Control"**

4. **Configura il nodo External Control**:
   - **IP Host**: `192.168.10.191` (IP AI Accelerator)
   - **Porta**: `50002`
   - **Timeout**: (default o 2.0)

5. **Salva il programma**

6. **STOP e poi PLAY di nuovo** il programma

7. **Verifica che la porta 50002 si apra**:
   ```bash
   ./FIX_PORTA_50002.sh
   ```

## 📚 Riferimenti

- **Repository GitHub**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **Issue #181**: Problemi con controllo remoto e dashboard
- **Issue #382**: Disconnessioni con reverse interface
- **Issue #965**: Problemi con controller su ROS2 Humble

## ⚠️ Problemi Noti dal Repository

Secondo le issue del repository GitHub:

1. **C218A1 Error**: External Control non configurato correttamente
2. **Segmentation fault**: Conflitto RTDE o versione driver
3. **Porta 50002 chiusa**: Programma non configurato correttamente
4. **Connection dropped**: Problemi con capacità real-time del sistema

## 🔧 Dopo aver configurato

Una volta che la porta 50002 è aperta:

```bash
./AVVIA_DRIVER_UFFICIALE.sh
```

Il driver dovrebbe avviarsi correttamente senza crash.









