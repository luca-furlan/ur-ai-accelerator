# 🏗️ Architettura Deployment - Vision + Robot System

## ✅ Architettura Corretta

```
┌─────────────────────────────────────────────────┐
│         AI ACCELERATOR (TUTTO QUI)              │
│                                                 │
│  ┌──────────────────────────────────────────┐  │
│  │  ROS2 Humble + MoveIt2                  │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                            │
│  ┌──────────────────────────────────────────┐  │
│  │  Camera Orbecc                          │  │
│  │  • RGB Feed                             │  │
│  │  • Depth Feed                           │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                            │
│  ┌──────────────────────────────────────────┐  │
│  │  Vision YOLO Detector                   │  │
│  │  • Object Detection YOLOv8              │  │
│  │  • 3D Coordinates                       │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                            │
│  ┌──────────────────────────────────────────┐  │
│  │  MoveIt Vision Controller               │  │
│  │  • Target Selection                     │  │
│  │  • Motion Planning                      │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                            │
│  ┌──────────────────────────────────────────┐  │
│  │  Vision Robot Coordinator               │  │
│  │  • State Management                     │  │
│  │  • Robot Control (RTDE)                 │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                            │
│  ┌──────────────────────────────────────────┐  │
│  │  Robot UR5e (192.168.10.194)           │  │
│  └──────────────────────────────────────────┘  │
│                                                 │
│  ┌──────────────────────────────────────────┐  │
│  │  WEB INTERFACE (Flask)                  │  │
│  │  • Port 5000                            │  │
│  │  • REST API                             │  │
│  │  • HTML Frontend                        │  │
│  └──────────────────────────────────────────┘  │
│                                                 │
│         IP: 192.168.1.100 (esempio)            │
└─────────────────────────────────────────────────┘
                    ↑
                    │ HTTP (browser)
                    │
        ┌───────────────────────┐
        │  WINDOWS (tua macchina)│
        │                       │
        │  Browser:             │
        │  http://192.168.1.100:5000 │
        └───────────────────────┘
```

## 📦 Cosa va dove

### AI Accelerator (Linux remoto)
- ✅ Tutti i file Python (vision_*.py, moveit_*.py, etc.)
- ✅ Web interface esistente (remote_ur_control/web_interface.py)
- ✅ Vision Web API (remote_ur_control/vision_web_api.py)
- ✅ Script bash (avvia_*.sh, ferma_*.sh)
- ✅ Launch files ROS2
- ✅ Documentazione

### Windows (tua macchina locale)
- ❌ Nessun codice da eseguire
- ✅ Solo browser web
- ✅ Opzionale: script deploy per trasferire file

## 🚀 Workflow

### 1. Deploy (una volta)
```bash
# Da Windows (con WSL o Git Bash)
bash deploy_vision_completo.sh

# Oppure manualmente via SCP/WinSCP
```

### 2. Avvio su AI Accelerator (ogni volta)
```bash
# SSH su AI Accelerator
ssh user@192.168.1.100

# Avvia tutto
cd ~/MekoAiAccelerator
bash avvia_sistema_completo.sh
```

### 3. Uso da Windows (solo browser)
```
Apri browser:
http://192.168.1.100:5000
```

## ✅ Vantaggi questa architettura

1. **Tutto centralizzato** - Un solo sistema da gestire
2. **Zero latenza** - ROS2 e robot sullo stesso network locale
3. **Semplice da Windows** - Solo browser, zero installazioni
4. **Scalabile** - Più client possono connettersi
5. **Sicuro** - Tutto isolato su AI Accelerator

## 🔧 Porte necessarie

- **5000** - Web Interface Flask
- **Opzionale 22** - SSH per deploy/debug




