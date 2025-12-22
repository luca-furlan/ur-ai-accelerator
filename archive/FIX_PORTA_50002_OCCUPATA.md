# 🔧 FIX: Porta 50002 "Address already in use"

## 🔴 PROBLEMA IDENTIFICATO

Il log mostra chiaramente:
```
[WARN] Failed to bind socket for port 50002 to address. Reason: Address already in use
```

**Causa**: Un altro processo sta usando la porta 50002, quindi il driver ROS2 non può aprirla.

---

## ✅ SOLUZIONE RAPIDA

### Opzione 1: Script Automatico (CONSIGLIATO)

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x libera_porta_50002.sh
./libera_porta_50002.sh
```

Questo script:
- ✅ Trova processo che usa porta 50002
- ✅ Kill processo driver ROS2 esistente
- ✅ Kill processo sulla porta 50002
- ✅ Verifica che porta sia libera
- ✅ Pronto per riavviare driver ROS2

**Poi avvia il driver:**
```bash
./avvia_driver_ros2.sh
```

---

### Opzione 2: Manuale

```bash
# 1. Trova processo sulla porta 50002
lsof -i :50002
# OPPURE
fuser 50002/tcp
# OPPURE
netstat -tulnp | grep 50002

# 2. Kill processo trovato
kill -9 <PID>

# 3. Kill driver ROS2 esistente
pkill -9 -f ur_ros2_control_node

# 4. Verifica porta libera
netstat -tuln | grep 50002
# Non dovresti vedere nulla

# 5. Riavvia driver ROS2
./avvia_driver_ros2.sh
```

---

### Opzione 3: Dalla Web Interface

Ho migliorato il sistema di avvio nella web interface:
- ✅ Verifica automaticamente processi sulla porta 50002
- ✅ Kill automaticamente processi che bloccano la porta
- ✅ Poi avvia il driver ROS2

**Clicca semplicemente "▶️ Avvia Driver ROS2" dalla web interface!**

---

## 🔍 VERIFICA CHE FUNZIONI

Dopo aver liberato la porta e avviato il driver:

```bash
./verifica_porta_50002.sh
```

**Dovresti vedere:**
```
✅ Porta 50002 APERTA
✅ Porta 50002 RAGGIUNGIBILE
✅ TUTTO OK: Driver attivo e porta 50002 aperta
```

---

## ⚠️ PERCHÉ SUCCEDE

La porta 50002 può rimanere occupata se:
1. Driver ROS2 precedente non è stato killato correttamente
2. Altro processo (es. vecchia istanza web interface) usa la porta
3. Processo zombie rimasto in memoria

**Soluzione**: Il nuovo sistema kill automaticamente tutti i processi prima di avviare il driver.

---

## ✅ RISULTATO

Dopo aver eseguito `libera_porta_50002.sh`:
- ✅ Porta 50002 libera
- ✅ Driver ROS2 fermo
- ✅ Pronto per riavviare tutto pulito

Poi avvia il driver ROS2 e la porta 50002 si aprirà correttamente!







