# 📘 CONFIGURAZIONE EXTERNAL CONTROL - GUIDA UFFICIALE

**Fonte**: Universal Robots Remote Operation Guide

## 🎯 Obiettivo

Configurare External Control URCap per permettere al driver ROS2 di controllare il robot tramite porta 50002.

## 📋 Passi Secondo Guida Ufficiale

### PASSO 1: Abilita Remote Control Mode

**Secondo Sezione 1.1 della guida:**

1. Vai sul **Teach Pendant**
2. **Hamburger Menu** (☰) → **Settings** → **System** → **Remote Control**
3. Premi **"Enable"**
4. Verifica che appaia il pulsante "Remote Control" in alto a destra (accanto al safety checksum)

**Nota dalla guida:**
> "several functions shared in this guide will require the robot to be in Remote Control mode"

### PASSO 2: Installa External Control URCap

1. Vai su: **Installation** → **URCaps**
2. Verifica che **"External Control"** sia installato
3. Se non installato, installalo

### PASSO 3: Crea Programma con External Control

1. **Crea nuovo programma** o apri `remote_control.urp`
2. **Aggiungi nodo "External Control"** al programma
3. **Configura il nodo**:
   - **IP Host**: `192.168.10.191` (IP AI Accelerator)
   - **Porta**: `50002` (porta standard per External Control)
   - **Timeout**: (default o 2.0)
4. **Salva il programma**

### PASSO 4: Avvia Programma

1. **STOP** il programma se è in esecuzione
2. **PLAY** il programma
3. Verifica che:
   - Il programma sia in **PLAYING**
   - "Remote Control" sia attivo
   - La porta 50002 si apra

## ✅ Verifica Configurazione

Esegui lo script di verifica:

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
chmod +x VERIFICA_REQUISITI_UFFICIALE.sh
./VERIFICA_REQUISITI_UFFICIALE.sh
```

**Dovresti vedere:**
- ✅ Remote Control abilitato
- ✅ Programma in PLAYING
- ✅ Porta 50002 APERTA

## 🔧 Risoluzione Problemi

### Porta 50002 ancora chiusa

1. **Verifica Remote Control**:
   ```bash
   # Via Dashboard
   echo "is in remote control" | nc 192.168.10.194 29999
   ```
   Deve rispondere: `true`

2. **Verifica programma**:
   - Il programma deve contenere il nodo "External Control"
   - Il nodo deve essere configurato con IP corretto (192.168.10.191)
   - Il programma deve essere in PLAYING

3. **Riavvia programma**:
   - STOP il programma
   - PLAY di nuovo
   - Attendi 2-3 secondi

4. **Verifica connessione di rete**:
   ```bash
   ping 192.168.10.194
   ```

## 📚 Riferimenti Guida Ufficiale

- **Sezione 1.1**: Setup and Requirements - Remote Control mode
- **Sezione 4.1**: Dashboard - Comandi di stato robot
- **Sezione 5**: Commanding the Robot - External Control per ROS2

## ⚠️ Note Importanti

1. **Remote Control obbligatorio**: Molte funzioni richiedono Remote Control mode abilitato
2. **Porta 50002**: Deve essere aperta quando External Control è attivo
3. **IP corretto**: Il nodo External Control deve puntare all'IP del PC che esegue ROS2 (192.168.10.191)
4. **Programma in PLAYING**: Il programma deve essere in esecuzione per aprire la porta

## 🚀 Dopo Configurazione

Una volta che la porta 50002 è aperta:

```bash
./AVVIA_DRIVER_UFFICIALE.sh
```

Il driver ROS2 dovrebbe connettersi correttamente senza crash.









