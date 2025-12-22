# 📁 ORGANIZZAZIONE REPOSITORY

## 🎯 Obiettivo
Rendere la repository minimale e organizzata, spostando file non essenziali in `archive/`.

## 📂 Struttura Finale

```
MekoAiAccelerator/
├── remote_ur_control/          # Codice principale
│   ├── web_interface.py        # Web interface principale
│   ├── remote_ur_controller.py # Controller robot
│   └── ...
├── scripts/                     # Script essenziali
│   ├── switch_controller.py    # Switch controller robusto
│   ├── attiva_controller.py    # Attiva controller
│   └── ...
├── archive/                     # File non essenziali (spostati qui)
│   ├── *.txt                   # File di documentazione vecchi
│   ├── *.md                    # Guide obsolete
│   └── ...
└── README.md                    # Documentazione principale
```

## 📋 File da Spostare in archive/

### Documentazione Obsoleta
- Tutti i file `*.txt` con soluzioni/guide vecchie
- File `*.md` duplicati o obsoleti
- File di troubleshooting non più necessari

### Script Non Essenziali
- Script di test non più usati
- Script di diagnostica vecchi
- Script duplicati

## ✅ File da Mantenere

### Essenziali
- `remote_ur_control/web_interface.py` - Web interface principale
- `remote_ur_control/remote_ur_controller.py` - Controller robot
- `switch_controller.py` - Switch controller robusto
- `attiva_controller.py` - Attiva controller
- `start_web_interface.py` - Avvia web interface
- `README.md` - Documentazione principale

### Utili ma Non Essenziali
- Script di setup iniziale
- Script di verifica base

## 🚀 Come Usare

1. **Avvia Web Interface:**
   ```bash
   python3 start_web_interface.py
   ```

2. **Switch Controller (se necessario):**
   ```bash
   python3 switch_controller.py forward_velocity_controller scaled_joint_trajectory_controller
   ```

3. **Attiva Controller:**
   ```bash
   python3 attiva_controller.py
   ```

## 📝 Note

- I file in `archive/` sono mantenuti per riferimento storico
- Non vengono più usati nel flusso principale
- Possono essere eliminati se non servono più
