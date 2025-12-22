# 📚 DOVE TROVARE LA DOCUMENTAZIONE UFFICIALE

## ❌ La Guida "Remote Operation Guide" NON Copre ROS2!

La guida che hai fornito ("Remote Operation Guide") **NON contiene** informazioni su External Control URCap per ROS2.

**Dalla guida stessa (Sezione 5):**
> "NOTE: Users choosing to utilize the official Universal Robots ROS driver should seek the Github page linked in the Reference section as it is outside the scope of this guide."

## ✅ DOVE TROVARE LA DOCUMENTAZIONE CORRETTA

### 1. Repository GitHub Ufficiale ROS2 Driver

**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

**Sezioni importanti:**
- **README.md**: Getting Started, Installation
- **ur_robot_driver/doc/**: Documentazione completa del driver
- **ur_robot_driver/doc/usage.rst**: Guida all'uso
- **ur_robot_driver/doc/setup/**: Setup e configurazione robot

### 2. Documentazione Online (se disponibile)

**URL**: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/

**Sezione**: Robot Setup / External Control URCap

### 3. Repository GitHub External Control URCap

**URL**: https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap

Questo repository contiene:
- File `.urcap` da installare
- Istruzioni di installazione
- Documentazione configurazione

### 4. README del Driver ROS2 (GitHub)

Nel README principale del repository ROS2 driver:

```markdown
## Getting Started

1. Install the driver
2. Start & Setup the robot
3. Start the driver
```

La sezione "Start & Setup the robot" dovrebbe contenere informazioni su External Control.

## 🔍 COSA CERCARE NELLA DOCUMENTAZIONE

Cerca queste parole chiave:
- "External Control URCap"
- "robot setup"
- "port 50002"
- "external_control"
- "URCap installation"
- "Teach Pendant configuration"

## 📋 COSA DICE LA DOCUMENTAZIONE (da ricerca web)

Secondo la documentazione trovata:

1. **External Control URCap deve essere installato** sul robot
2. **File URCap**: `externalcontrol-X.Y.Z.urcap` (es. `externalcontrol-1.0.5.urcap`)
3. **Porta**: 50002 (default)
4. **Configurazione**: IP Host del PC che esegue ROS2
5. **Programma**: Deve contenere nodo "External Control" e essere in PLAYING

## 🎯 DOVE LEGGERE ORA

1. **GitHub Repository ROS2 Driver**:
   - Vai su: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
   - Leggi: README.md
   - Cerca: "robot setup" o "External Control"

2. **Documentazione nel repository**:
   - Cartella: `ur_robot_driver/doc/`
   - File: `setup/robot_setup.rst` o simile

3. **Issues GitHub**:
   - Cerca issues con tag "External Control" o "port 50002"
   - Vedi soluzioni proposte da altri utenti

## ⚠️ NOTA IMPORTANTE

La guida "Remote Operation Guide" che hai fornito copre:
- ✅ RTDE (porta 30004)
- ✅ Dashboard (porta 29999)
- ✅ Primary/Secondary Interface (porte 30001/30002)
- ✅ Interpreter Mode (porta 30020)
- ✅ Socket TCP/IP
- ❌ **NON copre ROS2 Driver o External Control URCap**

Per ROS2, devi consultare la documentazione del driver ROS2 su GitHub.









