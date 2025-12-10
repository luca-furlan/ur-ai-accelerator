# 🎮 GUIDA JOYSTICK WEB INTERFACE - MOVIMENTI LENTI

## ✅ COSA È STATO FATTO

- ✅ Velocità joystick **RIDOTTE** per movimenti lenti e sicuri:
  - Joint mode: **0.05 rad/s** (era 0.1 rad/s)
  - Cartesian mode: **0.01 m/s = 10mm/s** (era 20mm/s)
  - Rotazione: **0.1 rad/s** (era 0.2 rad/s)

---

## 🚀 AVVIO WEB INTERFACE

### Opzione 1: Script automatico (consigliato)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
./avvia_web_interface_joystick.sh
```

### Opzione 2: Manuale

```bash
ssh lab@192.168.10.191
export UR_ROBOT_IP=192.168.10.194
export WEB_PORT=8080
export WEB_HOST=0.0.0.0
cd ~/MekoAiAccelerator
python3 -m remote_ur_control.web_interface
```

---

## 🌐 ACCEDI ALL'INTERFACCIA

Apri il browser e vai su:

```
http://192.168.10.191:8080
```

Oppure dalla macchina locale:

```
http://<IP_AI_ACCELERATOR>:8080
```

---

## 🎮 COME USARE I JOYSTICK

### Joystick 1 (XY) - Sinistra
- **X (orizzontale)**: Muove joint 1 (base) o Tool Y (in cartesian mode)
- **Y (verticale)**: Muove joint 0 (base) o Tool X (in cartesian mode)

### Joystick 2 (Z/Rotation) - Destra
- **X (orizzontale)**: Rotazione Tool (Rz) in cartesian mode
- **Y (verticale)**: Muove Tool Z (su/giù) in cartesian mode

### Modalità

**Joint Mode (default):**
- Joystick 1: Controlla joint 0 e 1
- Joystick 2: Non usato in questa modalità
- Velocità: **0.05 rad/s** (molto lento e sicuro)

**Cartesian Mode (checkbox attiva):**
- Joystick 1: Controlla Tool X e Y
- Joystick 2: Controlla Tool Z e rotazione Rz
- Velocità: **10mm/s** (molto lento e sicuro)

---

## ⚠️ SICUREZZA

1. **Prima di iniziare:**
   - Verifica che l'area di lavoro sia libera
   - Tieni pronto l'e-stop fisico del robot
   - Inizia con movimenti piccoli

2. **Durante l'uso:**
   - Muovi il joystick lentamente
   - I movimenti sono già limitati a velocità basse
   - Rilascia il joystick per fermare il movimento

3. **Emergency Stop:**
   - Usa il pulsante "Emergency Stop" nell'interfaccia
   - Oppure usa l'e-stop fisico del robot

---

## 🔧 FUNZIONAMENTO TECNICO

### Come funziona

1. **Joystick → Velocità:**
   - Il joystick invia velocità target al robot
   - Le velocità sono pubblicate a **125Hz** via ROS2
   - Il robot esegue movimenti fluidi e continui

2. **ROS2 Bridge:**
   - La web interface usa il ROS2 bridge per inviare comandi
   - I comandi vanno al topic `/forward_velocity_controller/commands`
   - Il driver ROS2 traduce in comandi robot

3. **Deadzone:**
   - C'è una deadzone di **0.15** per evitare micro-movimenti
   - Movimenti molto piccoli vengono ignorati

---

## 📋 CHECKLIST PRIMA DI USARE

- [ ] Driver ROS2 attivo (`ur_ros2_control_node` in esecuzione)
- [ ] Robot connesso (vedi "Robot Status" nell'interfaccia)
- [ ] Robot in RUNNING
- [ ] Programma in PLAYING
- [ ] Remote Control abilitato
- [ ] Area di lavoro libera
- [ ] E-stop fisico accessibile

---

## 🐛 TROUBLESHOOTING

### Il robot non si muove

1. **Verifica connessione:**
   - Controlla sezione "Robot Status" nell'interfaccia
   - Verifica che "Remote Control" sia "true"
   - Verifica che "Program State" sia "PLAYING"

2. **Verifica driver ROS2:**
   ```bash
   ps aux | grep ur_ros2_control_node
   ros2 topic list | grep joint_states
   ```

3. **Verifica log:**
   - Controlla la console del browser (F12)
   - Cerca errori nella sezione "ROS2 Monitor"

### Movimenti troppo veloci/lenti

- Le velocità sono già ridotte per sicurezza
- Se vuoi ancora più lento, modifica in `web_interface.py`:
  - `JOY_MAX = 0.05` → riduci a `0.02` o `0.01`
  - `JOY_CART_VEL = 0.01` → riduci a `0.005`

### Joystick non risponde

1. **Verifica JavaScript:**
   - Apri console browser (F12)
   - Cerca errori JavaScript

2. **Verifica connessione:**
   - Controlla che la web interface sia raggiungibile
   - Verifica che non ci siano errori di rete

---

## ✅ TEST CONSIGLIATO

1. **Test iniziale:**
   - Avvia web interface
   - Verifica che "Robot Status" mostri tutto OK
   - Muovi joystick 1 leggermente a destra
   - Il robot dovrebbe muoversi molto lentamente
   - Rilascia il joystick → il robot si ferma

2. **Test cartesian mode:**
   - Attiva checkbox "Cartesian mode"
   - Muovi joystick 1 → Tool si muove in X/Y
   - Muovi joystick 2 → Tool si muove in Z e ruota

---

## 🎯 PROSSIMI PASSI

Una volta che i joystick funzionano:

1. ✅ Test movimenti base
2. ✅ Test movimenti cartesiani
3. ✅ Test precisione e controllo fine
4. ✅ Integrazione con altri sistemi (se necessario)

---

## 📞 SUPPORTO

Se hai problemi:
1. Controlla la sezione "ROS2 Monitor" nell'interfaccia
2. Controlla la sezione "Robot Status"
3. Verifica log del driver ROS2
4. Verifica console browser (F12)

**Buon controllo! 🎮🤖**

