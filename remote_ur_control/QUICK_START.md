# Quick Start - Controllo Robot UR

## Problema: Robot non si muove

Se i comandi vengono inviati ma il robot non si muove, controlla:

### 1. Stato Robot (Dashboard Server)

```bash
python -m remote_ur_control.check_robot_status --robot-ip 192.168.10.194
```

**Stati richiesti:**
- `robotmode`: `RUNNING` (non `POWER_OFF` o `IDLE`)
- `programState`: `PLAYING` (non `STOPPED`)
- `is in remote control`: `true`

### 2. Abilitazione Robot

**Opzione A - Manuale (Teach Pendant):**
1. Accendi il robot (se spento)
2. Carica o crea un programma
3. Imposta il programma in modalità **REMOTE**
4. Premi **PLAY** sul teach pendant
5. Verifica che il programma sia in esecuzione

**Opzione B - Automatica (se supportato):**
```bash
python -m remote_ur_control.enable_robot --robot-ip 192.168.10.194
```

### 3. Test Movimento

Dopo che il robot è pronto:

```bash
# Test rapido
python -m remote_ur_control.test_all_commands

# Test completo con diagnostica
python -m remote_ur_control.test_movement --robot-ip 192.168.10.194 --auto
```

### 4. Web Interface

Avvia l'interfaccia web:

```bash
python -m remote_ur_control.launch_ur_control --robot-ip 192.168.10.194
```

Poi apri il browser su `http://<AI_ACCELERATOR_IP>:8080`

## Troubleshooting

### Robot in POWER_OFF
- **Causa**: Robot spento
- **Soluzione**: Accendi il robot sul teach pendant o usa `enable_robot.py`

### Programma STOPPED
- **Causa**: Nessun programma in esecuzione
- **Soluzione**: Avvia un programma sul teach pendant in modalità REMOTE

### Remote Control = false
- **Causa**: Programma non in modalità REMOTE
- **Soluzione**: Imposta il programma in modalità REMOTE sul teach pendant

### Comandi inviati ma nessun movimento
- Verifica che il programma sul teach pendant sia **PLAYING**
- Verifica che non ci siano errori sul teach pendant
- Controlla che il robot non sia in modalità SAFE STOP
- Verifica che i valori di movimento non siano troppo piccoli

## Note Importanti

⚠️ **SICUREZZA**:
- Assicurati che non ci siano operatori nell'area di lavoro
- Mantieni l'e-stop facilmente raggiungibile
- Inizia sempre con movimenti piccoli e sicuri

📝 **Modalità REMOTE**:
- Il programma sul teach pendant DEVE essere in modalità REMOTE
- Solo in REMOTE il robot accetta comandi via socket (porta 30002)
- In modalità LOCAL i comandi vengono ignorati

