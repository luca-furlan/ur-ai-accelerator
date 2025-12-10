# 🚀 Wizard Setup Guidato - Flusso Sicuro e Solido

## 📋 Panoramica

Il nuovo **Wizard Setup Guidato** nella web interface gestisce automaticamente tutto il processo di setup del robot in modo sicuro e solido, passo dopo passo.

## 🎯 Flusso Automatico

### **Step A: Avvia Driver ROS2**
- ✅ Kill automatico di processi esistenti
- ✅ Verifica porta 50002 libera
- ✅ Avvio driver ROS2 in background
- ✅ Attesa stabilizzazione

### **Step B: Verifica Driver Attivo**
- ✅ Verifica che driver ROS2 sia attivo
- ✅ Verifica che porta 50002 sia aperta
- ✅ Polling automatico ogni 2 secondi
- ✅ Max 10 tentativi (20 secondi)

### **Step C: Attiva Controller**
- ✅ Attiva automaticamente `forward_velocity_controller`
- ✅ Evita crash con `scaled_joint_trajectory_controller`
- ✅ Verifica attivazione corretta

### **Step D: Attiva External Control sul Teach Pendant**
- 📋 Istruzioni chiare visualizzate
- ✅ Pulsante "Ho Attivato External Control" appare
- ⏳ Attesa azione utente

### **Step E: Verifica Connessione**
- ✅ Verifica robot in modalità RUNNING
- ✅ Verifica safety mode NORMAL
- ✅ Verifica Remote Control attivo o programma PLAYING
- ✅ Polling automatico ogni 2 secondi
- ✅ Max 15 tentativi (30 secondi)

## 🎨 Interfaccia

- **Step completati**: Bordo verde, background chiaro
- **Step attivo**: Bordo blu, background azzurro
- **Step bloccati**: Opacità ridotta, bordo grigio
- **Status badge**: 
  - ✅ Completato (verde)
  - 🔄 In corso (giallo)
  - ❌ Errore (rosso)
  - ⏸️ Bloccato (grigio)

## 🔄 Come Usare

1. **Apri web interface**: `http://192.168.10.191:8080`
2. **Vai alla sezione "🚀 Setup Guidato Robot"**
3. **Clicca "▶️ Avvia Driver ROS2"** nello Step A
4. **Attendi completamento automatico** degli step B e C
5. **Segui le istruzioni** nello Step D per attivare External Control sul Teach Pendant
6. **Clicca "✅ Ho Attivato External Control"** quando fatto
7. **Attendi verifica connessione** nello Step E
8. **✅ Pronto!** Puoi controllare il robot con i joystick

## 🛡️ Sicurezza

- ✅ **Kill automatico** processi duplicati
- ✅ **Verifica porta** prima di avviare
- ✅ **Controller sicuro** (`forward_velocity_controller` invece di `scaled_joint_trajectory_controller`)
- ✅ **Verifica stato** robot prima di permettere controllo
- ✅ **Timeout** su tutte le operazioni
- ✅ **Messaggi di errore** chiari

## 🔧 Troubleshooting

### Driver non si avvia
- Verifica che non ci siano altri processi ROS2 attivi
- Controlla log: `tail -50 /tmp/ros2_driver.log`
- Usa "🔍 Verifica Processi" per kill manuale

### Porta 50002 non si apre
- Verifica che nessun altro processo usi la porta
- Riavvia driver: ferma e riavvia dallo Step A

### Controller non si attiva
- Verifica che driver ROS2 sia attivo
- Controlla che robot sia in modalità RUNNING
- Prova switch manuale controller

### Robot non si connette
- Verifica che External Control sia configurato correttamente sul Teach Pendant
- IP Host: `192.168.10.191`
- Porta: `50002`
- Verifica che programma sia in stato PLAYING

## 📊 Status Monitor

La sezione "Status Grid" mostra sempre:
- **Driver ROS2**: Attivo/Fermo
- **Controller**: Nome controller attivo
- **Robot Mode**: Modalità robot (RUNNING/IDLE/etc)
- **Porta 50002**: Aperta/Chiusa

## 🎯 Vantaggi

1. ✅ **Flusso guidato** - Nessuna confusione su cosa fare
2. ✅ **Automatico** - Meno interventi manuali
3. ✅ **Sicuro** - Verifiche a ogni step
4. ✅ **Solido** - Gestione errori e timeout
5. ✅ **Chiaro** - Messaggi e status visibili
6. ✅ **Robusto** - Kill automatico processi duplicati

## 🔄 Reset Wizard

Per riavviare il wizard:
1. Ferma driver: "⏹️ Ferma Driver"
2. Ricarica pagina
3. Ricomincia dallo Step A

