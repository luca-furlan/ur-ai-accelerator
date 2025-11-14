# Diagnostica Spazio Disco - AI Accelerator

## 📊 STATO DISCO PRINCIPALE

**Disco**: `/dev/nvme0n1p1`
- **Totale**: 937 GB
- **Usato**: 98 GB (12%)
- **Disponibile**: 791 GB
- **Stato**: ✅ **OTTIMO** (abbondante spazio disponibile)

## 📁 CARTELLE PIÙ GRANDI IN `/home/lab`

| Cartella | Dimensione | Note |
|----------|------------|------|
| `~/pandai_ark` | **8.0 GB** | Progetto principale |
| `~/projects` | **2.7 GB** | Altri progetti |
| `~/ros2_ws` | **727 MB** | Workspace ROS2 |
| `~/snap` | **439 MB** | Pacchetti Snap |
| `~/Downloads` | **223 MB** | Download |
| `~/.cache` | **128 MB** | Cache sistema |
| `~/.local` | **35 MB** | Dati locali |
| `~/MekoAiAccelerator` | **440 KB** | Progetto corrente |

## 🔍 ANALISI DETTAGLIATA

### 1. `~/pandai_ark` (8.0 GB)
- **Sottocartelle principali**:
  - `~/pandai_ark/ros`: **5.7 GB** (modelli ML: 1.2 GB)
  - `~/pandai_ark/polyscope`: **2.4 GB** (node_modules: ~30 MB totali)
- **File grandi trovati**: modelli ML (.safetensors, .pt), file tar di build
- **Possibilità di pulizia**: rimuovere modelli ML non utilizzati, build artifacts vecchi

### 2. `~/projects` (2.7 GB)
- **Sottocartelle**:
  - `ur-ai-accelerator-advanced-vision`: **2.6 GB** (progetto principale)
  - `ur-ai-accelerator-object-recognition`: **28 MB**
  - `ur-ai-accelerator-remote`: **2.0 MB**
  - `ur-ai-accelerator-streaming`: **476 KB**

### 3. `~/ros2_ws` (727 MB)
- Workspace ROS2 con pacchetti compilati
- **Normale** per workspace ROS2 con build

### 4. Cache e temporanei
- `~/.cache`: 128 MB totali
  - `node-gyp`: 56 MB
  - `pip`: 34 MB
  - `tracker3`: 20 MB
  - Altri: ~18 MB
- `~/.ros`: 5.2 MB (normale per ROS)
- `~/Downloads`: 223 MB (contiene file .deb: cursor 104MB + 100MB, aikit 20MB)

## 💡 PROPOSTE DI PULIZIA

### ✅ PRIORITÀ BASSA (spazio già abbondante)
Lo spazio disponibile è ottimo (791 GB liberi), quindi la pulizia non è urgente. Tuttavia:

#### 🚀 PULIZIA RAPIDA (Script automatico)
```bash
# Trasferisci lo script sull'AI Accelerator
scp cleanup_ai_accelerator.sh lab@192.168.10.191:~/

# Esegui sull'AI Accelerator
ssh lab@192.168.10.191
bash cleanup_ai_accelerator.sh
```

#### 📋 PULIZIA MANUALE

1. **Pulizia cache Python** (~pochi MB):
   ```bash
   ssh lab@192.168.10.191
   find ~ -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
   find ~ -name "*.pyc" -delete 2>/dev/null
   ```

2. **Pulizia cache pip e node-gyp** (~90 MB):
   ```bash
   ssh lab@192.168.10.191
   pip cache purge 2>/dev/null || python3 -m pip cache purge
   rm -rf ~/.cache/node-gyp/*
   ```

3. **Pulizia file .deb in Downloads** (~224 MB):
   ```bash
   ssh lab@192.168.10.191
   rm ~/Downloads/*.deb  # Rimuove cursor e aikit installati
   ```

4. **Pulizia log ROS2 vecchi** (~pochi MB):
   ```bash
   ssh lab@192.168.10.191
   find ~/.ros/log -type f -mtime +30 -delete
   find ~/ros2_ws/log -type f -mtime +30 -delete
   ```

5. **Pulizia modelli ML non utilizzati** (potenziale ~1.2 GB):
   ```bash
   ssh lab@192.168.10.191
   # Verifica quali modelli sono utilizzati prima di rimuovere!
   ls -lh ~/pandai_ark/ros/data/models/*/checkpoint-*/
   # Rimuovi solo checkpoint vecchi se sicuro
   ```

6. **Pulizia build artifacts** (se non servono):
   ```bash
   ssh lab@192.168.10.191
   # Verifica prima cosa c'è
   du -sh ~/pandai_ark/polyscope/urcaps/*/target
   du -sh ~/pandai_ark/polyscope/urcaps/*/dist
   ```

7. **Pulizia pacchetti Snap non utilizzati** (~potenziale centinaia MB):
   ```bash
   ssh lab@192.168.10.191
   snap list --all | awk '/disabled/{print $1, $3}'
   # Rimuovi manualmente quelli non necessari
   ```

### 📋 COMANDI DI VERIFICA

```bash
# Verifica spazio totale
ssh lab@192.168.10.191 "df -h"

# Analizza cartelle più grandi
ssh lab@192.168.10.191 "du -sh ~/* 2>/dev/null | sort -hr | head -20"

# Cerca file grandi (>100MB)
ssh lab@192.168.10.191 "find ~ -type f -size +100M 2>/dev/null"

# Verifica node_modules
ssh lab@192.168.10.191 "find ~ -type d -name node_modules -exec du -sh {} \; 2>/dev/null | sort -hr"

# Verifica ambienti virtuali Python
ssh lab@192.168.10.191 "find ~ -type d \( -name venv -o -name .venv -o -name env \) -exec du -sh {} \; 2>/dev/null | sort -hr"
```

## ✅ CONCLUSIONE

**Lo spazio su disco dell'AI Accelerator è OTTIMO:**
- ✅ 791 GB disponibili (84% libero)
- ✅ Nessuna urgenza di pulizia
- ✅ Sistema funzionante normalmente

**Raccomandazioni:**
- Monitorare periodicamente lo spazio (soprattutto se `~/pandai_ark` cresce)
- Pulire cache e temporanei ogni 3-6 mesi
- Rimuovere progetti non più utilizzati se lo spazio scende sotto 500 GB

