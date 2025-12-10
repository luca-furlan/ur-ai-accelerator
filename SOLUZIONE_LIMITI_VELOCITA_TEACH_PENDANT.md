# 🔧 SOLUZIONE DEFINITIVA: Anomalia Velocità Giunti - Limiti Teach Pendant

## 🔴 PROBLEMA REALE

Il robot va in anomalia perché i **limiti di velocità dei giunti configurati sul Teach Pendant** vengono superati.

Secondo il manuale UR5e:
- I robot UR hanno **limiti di velocità configurabili** per ciascun giunto
- Tolleranza: **1,15°/s** (circa **0.02 rad/s**)
- Se questi limiti vengono superati, il robot va in **anomalia di sicurezza**

---

## 🎯 CAUSA PRINCIPALE

**I limiti di velocità sul Teach Pendant sono troppo bassi o la velocità inviata li supera.**

Anche velocità basse come **0.05 rad/s** (circa 2.87°/s) possono superare limiti configurati a **1,15°/s**.

---

## ✅ SOLUZIONE IMMEDIATA

### Passo 1: Verifica Limiti Velocità sul Teach Pendant

**Sul Teach Pendant:**
1. Vai su **Settings** → **Safety** → **Joint Limits**
2. Controlla **Speed Limits** per ciascun giunto
3. **AUMENTA** i limiti se sono troppo bassi (es. da 1,15°/s a almeno 10°/s)

**OPPURE:**

1. Vai su **Settings** → **Safety** → **Speed Limits**
2. Verifica limiti configurabili
3. Assicurati che siano almeno **10°/s** (circa **0.17 rad/s**) per permettere movimento

---

### Passo 2: Usa Velocità MOLTO BASSA

**Test con velocità sotto la tolleranza (0.01 rad/s = 0.57°/s):**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test con velocità MOLTO BASSA (0.01 rad/s = 0.57°/s)
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# Ferma
ros2 topic pub --once /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

---

### Passo 3: Modifica Web Interface per Velocità Ancora Più Bassa

**Nel file `web_interface.py`, cambia:**

```javascript
const JOY_MAX = 0.01;  // max joint velocity (rad/s) - MOLTO BASSO per evitare anomalie
```

**Questo corrisponde a circa 0.57°/s, sotto la tolleranza di 1,15°/s.**

---

## 📋 PROCEDURA COMPLETA

### 1. Risolvi Anomalia
- Sul Teach Pendant: **Reset anomalia**
- Riavvia programma

### 2. Verifica/Aumenta Limiti Velocità
- **Settings** → **Safety** → **Speed Limits**
- Aumenta limiti a almeno **10°/s** (0.17 rad/s) per ciascun giunto
- Salva modifiche

### 3. Test con Velocità Molto Bassa
```bash
cd ~/MekoAiAccelerator
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test 0.01 rad/s (0.57°/s)
timeout 3 ros2 topic pub -r 10 /forward_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray \
    '{data: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### 4. Se Funziona, Aumenta Gradualmente
- Se 0.01 rad/s funziona, prova 0.02 rad/s
- Poi 0.05 rad/s
- Poi 0.1 rad/s
- **SEMPRE sotto i limiti configurati sul Teach Pendant**

---

## 🎯 CONVERSIONE VELOCITÀ

- **1 rad/s = 57.3°/s**
- **0.01 rad/s = 0.57°/s** (sotto tolleranza 1,15°/s)
- **0.02 rad/s = 1.15°/s** (al limite tolleranza)
- **0.05 rad/s = 2.87°/s** (sopra tolleranza se limite è 1,15°/s)
- **0.1 rad/s = 5.73°/s**
- **0.17 rad/s = 10°/s** (raccomandato minimo)

---

## ⚠️ IMPORTANTE

**Il problema NON è EtherNet/IP (già disattivato).**

**Il problema è:**
1. **Limiti di velocità sul Teach Pendant troppo bassi**
2. **Velocità inviata supera i limiti configurati**

**Soluzione:**
1. **Aumenta limiti velocità sul Teach Pendant** (almeno 10°/s)
2. **OPPURE usa velocità molto bassa** (0.01 rad/s) per iniziare

---

## 🔍 VERIFICA LIMITI ATTUALE

**Sul Teach Pendant:**
- Vai su **Settings** → **Safety** → **Speed Limits**
- Leggi i limiti configurati per ciascun giunto
- Se sono **1,15°/s** o meno, **AUMENTALI** a almeno **10°/s**

---

## 📞 SE ANCORA NON FUNZIONA

1. Verifica log errori specifici:
   ```bash
   tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -i "speed\|velocity\|limit\|anomaly"
   ```

2. Controlla messaggi di errore sul Teach Pendant:
   - Cosa dice esattamente l'anomalia?
   - Quale giunto va in anomalia?
   - Quale velocità stavi usando?

3. Contatta supporto Universal Robots con:
   - Limiti velocità configurati
   - Velocità che causa anomalia
   - Messaggio errore esatto

