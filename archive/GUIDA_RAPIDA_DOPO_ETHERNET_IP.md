# 🚀 GUIDA RAPIDA - DOPO AVER DISABILITATO ETHERNET/IP

## ✅ COSA HAI FATTO

Hai disabilitato EtherNet/IP sul robot UR. Questo è il passo corretto per risolvere il problema RTDE overflow.

---

## 📋 VERIFICA STATO ATTUALE

Esegui questo comando sulla macchina remota (AI Accelerator):

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 verifica_stato_dopo_ethernet_ip.py
```

Lo script verificherà:
- ✅ Connessione robot
- ✅ Dashboard Server (porta 29999)
- ✅ Robot Mode (deve essere RUNNING)
- ✅ Program State (deve essere PLAYING o STOP)
- ✅ Remote Control (deve essere abilitato)
- ✅ RTDE disponibile (senza overflow)
- ✅ Driver ROS2 attivo (se già avviato)
- ✅ Porta 50002 in ascolto

---

## 🎯 PROSSIMI PASSI

### Passo 1: Verifica sul Teach Pendant

**IMPORTANTE:** Dopo aver disabilitato EtherNet/IP, devi:

1. **Riavviare il robot** (power cycle completo)
   - Spegni e riaccendi il robot
   - Attendi che si avvii completamente

2. **Verifica configurazione:**
   - Installation → Fieldbus
   - EtherNet/IP: **DISABILITATO** ✅
   - PROFINET: **DISABILITATO** ✅

3. **Abilita Remote Control:**
   - Settings → System → Remote Control
   - Deve essere **ABILITATO** ✅

4. **Crea programma con External Control:**
   - Crea/modifica programma
   - Aggiungi nodo "External Control"
   - Configura:
     - Host IP: `192.168.10.191`
     - Port: `50002`
     - Host Name: (lascia vuoto)
   - **NON avviare ancora** (lascia in STOP)

---

### Passo 2: Verifica stato con script

```bash
python3 verifica_stato_dopo_ethernet_ip.py
```

Lo script ti dirà cosa è OK e cosa manca.

---

### Passo 3: Avvia driver ROS2

**SOLO DOPO** che tutti i check sono OK:

```bash
# Opzione 1: Usa lo script bash completo
cd ~/MekoAiAccelerator/metodo_guida_pratica
./VERIFICA_E_AVVIA_DOPO_ETHERNET_IP.sh

# Opzione 2: Avvia manualmente
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.10.194 \
    launch_rviz:=false
```

---

### Passo 4: Verifica che funzioni

Nel log del driver ROS2 dovresti vedere:

✅ **Messaggi positivi:**
- `System successfully started!`
- `Robot connected`
- Nessun errore RTDE overflow

❌ **NON dovresti vedere:**
- `Pipeline producer overflowed!`
- `RTDE overflow`
- `Segmentation fault`
- `Connection refused`

---

### Passo 5: Connetti il robot

**SOLO DOPO** che il driver ROS2 è avviato e funzionante:

1. Sul Teach Pendant:
   - Verifica che External Control sia configurato correttamente
   - Verifica che il programma sia in STOP
   - Premi **PLAY**

2. Il robot dovrebbe connettersi al PC:
   - Vedrai nel log del driver: `Robot connected`
   - Il robot passerà in modalità REMOTE CONTROL

---

## 🔍 TROUBLESHOOTING

### Se vedi ancora "RTDE overflow"

1. **Verifica EtherNet/IP:**
   - Sul Teach Pendant: Installation → Fieldbus
   - Assicurati che sia **DISABILITATO**
   - Riavvia il robot dopo aver disabilitato

2. **Verifica altri processi RTDE:**
   ```bash
   # Ferma tutto
   pkill -f 'ur_robot_driver|web_interface|external_control_proxy'
   
   # Verifica porta RTDE libera
   netstat -tuln | grep 30004
   ```

3. **Verifica driver ROS2:**
   - Assicurati che solo il driver ROS2 usi RTDE
   - Non avviare web interface mentre il driver ROS2 è attivo

---

### Se il robot non si connette

1. **Verifica porta 50002:**
   ```bash
   netstat -tuln | grep 50002
   # Dovresti vedere: "0.0.0.0:50002" in LISTEN
   ```

2. **Verifica configurazione External Control:**
   - Host IP: `192.168.10.191` (IP del PC)
   - Port: `50002`
   - Host Name: (vuoto)

3. **Verifica Remote Control:**
   - Settings → System → Remote Control
   - Deve essere abilitato

---

## 📊 CHECKLIST FINALE

Prima di avviare il driver ROS2:

- [ ] EtherNet/IP disabilitato sul robot
- [ ] PROFINET disabilitato sul robot
- [ ] Robot riavviato dopo disabilitazione
- [ ] Remote Control abilitato
- [ ] Programma con External Control creato (ma in STOP)
- [ ] Configurazione External Control corretta (IP: 192.168.10.191, Port: 50002)

Dopo aver avviato il driver ROS2:

- [ ] Driver ROS2 avviato senza errori
- [ ] Porta 50002 in ascolto
- [ ] Nessun errore RTDE overflow nel log
- [ ] Nessun segmentation fault

Quando tutto è OK:

- [ ] Premi PLAY sul Teach Pendant
- [ ] Robot si connette al PC
- [ ] Nessun errore nel log

---

## 🆘 AIUTO

Se hai problemi:

1. Esegui lo script di verifica:
   ```bash
   python3 verifica_stato_dopo_ethernet_ip.py
   ```

2. Controlla i log del driver ROS2:
   ```bash
   tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log
   ```

3. Verifica lo stato del robot via web interface:
   ```bash
   # Avvia web interface (in un altro terminale)
   export UR_ROBOT_IP=192.168.10.194
   python3 -m remote_ur_control.web_interface
   
   # Apri browser: http://192.168.10.191:8080
   # Vai alla sezione "Robot Status"
   ```

---

## ✅ SUCCESSO

Se tutto funziona:

- ✅ Driver ROS2 avviato senza errori
- ✅ Robot connesso
- ✅ Nessun RTDE overflow
- ✅ Puoi controllare il robot via ROS2

🎉 **Congratulazioni! Il sistema è pronto.**







