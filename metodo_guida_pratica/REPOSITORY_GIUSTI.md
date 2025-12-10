# 📚 Repository GitHub Universal Robots - Quali Usare

## 🎯 Obiettivo: Controllare UR5e con ROS2

Per controllare un robot UR5e reale con ROS2, servono questi repository:

## ✅ Repository PRINCIPALI (Obbligatori)

### 1. **Universal_Robots_ROS2_Driver** ⭐ PRINCIPALE
**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

**Cosa fa:**
- Driver ROS2 principale per controllare robot UR
- Gestisce comunicazione con robot
- Fornisce controller ROS2 (trajectory, velocity, etc.)
- **Questo è il repository più importante!**

**Cosa contiene:**
- `ur_robot_driver` - Driver hardware
- `ur_controllers` - Controller ROS2
- Launch files (`ur_control.launch.py`)
- Documentazione

**Come installare:**
```bash
sudo apt-get install ros-humble-ur
# OPPURE compila da sorgente
```

**Uso:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194
```

---

### 2. **Universal_Robots_ExternalControl_URCap** ⭐ NECESSARIO
**URL**: https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap

**Cosa fa:**
- URCap da installare sul robot
- Permette al driver ROS2 di connettersi al robot
- Apre porta 50002 per controllo esterno
- **Senza questo, il driver ROS2 non può connettersi!**

**Cosa contiene:**
- File `.urcap` da installare sul Teach Pendant
- Versione: `externalcontrol-1.0.5.urcap` (per Polyscope 5.x)
- Versione: `externalcontrol-X.X.X.urcapx` (per Polyscope X)

**Come installare:**
1. Scarica file `.urcap` dalla pagina Releases
2. Copia su USB
3. Installa sul Teach Pendant: Installation → URCaps → +

**Download diretto:**
```bash
wget https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.urcap
```

---

## 📖 Repository UTILI (Opzionali ma Consigliati)

### 3. **Universal_Robots_ROS2_Documentation**
**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Documentation

**Cosa fa:**
- Documentazione completa del driver ROS2
- Guide di setup
- Esempi di utilizzo

**Quando usare:**
- Per approfondire funzionalità
- Per troubleshooting
- Per esempi avanzati

---

### 4. **Universal_Robots_ROS2_Tutorials**
**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials

**Cosa fa:**
- Tutorial pratici
- Esempi di codice
- Guide passo-passo

**Quando usare:**
- Per imparare come usare il driver
- Per vedere esempi pratici
- Per capire best practices

---

## 🎮 Repository per SIMULAZIONE (Non Necessari per Robot Reale)

### 5. **Universal_Robots_ROS2_GZ_Simulation**
**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation

**Cosa fa:**
- Simulazione robot UR con Gazebo
- Per test senza robot fisico

**Quando usare:**
- Solo se vuoi simulare senza robot reale
- Per test di codice prima di usare robot fisico

---

### 6. **Universal_Robots_ROS2_Gazebo_Simulation**
**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation

**Cosa fa:**
- Simulazione con Gazebo Classic
- Alternativa alla simulazione GZ

**Quando usare:**
- Solo per simulazione
- Non necessario per robot reale

---

## 🔧 Repository TECNICI (Per Sviluppatori)

### 7. **Universal_Robots_Client_Library**
**URL**: https://github.com/UniversalRobots/Universal_Robots_Client_Library

**Cosa fa:**
- Libreria C++ di basso livello
- Usata dal driver ROS2 internamente
- Per sviluppatori avanzati

**Quando usare:**
- Solo se sviluppi driver personalizzati
- Non necessario per uso normale

---

### 8. **Universal_Robots_ROS2_Description**
**URL**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

**Cosa fa:**
- File URDF per robot UR
- Descrizioni geometriche
- Usato da MoveIt2

**Quando usare:**
- Se usi MoveIt2 per pianificazione
- Per visualizzazione in RViz
- Non necessario per controllo base

---

## 📋 RIEPILOGO - Cosa Ti Serve

### Per Robot Reale UR5e con ROS2:

1. ✅ **Universal_Robots_ROS2_Driver** - Driver principale (INSTALLATO)
2. ✅ **Universal_Robots_ExternalControl_URCap** - URCap da installare sul robot (DA SCARICARE)

### Opzionali ma Utili:

3. 📖 **Universal_Robots_ROS2_Documentation** - Documentazione
4. 📚 **Universal_Robots_ROS2_Tutorials** - Tutorial ed esempi

### Non Necessari (per ora):

- ❌ Repository di simulazione (solo se vuoi simulare)
- ❌ Client Library (solo per sviluppatori avanzati)
- ❌ ROS2 Description (solo se usi MoveIt2)

## 🎯 CONCLUSIONE

**Repository GIUSTI per te:**

1. **Universal_Robots_ROS2_Driver** - ✅ Già installato
2. **Universal_Robots_ExternalControl_URCap** - ⚠️ Da scaricare e installare sul robot

Questi due sono sufficienti per controllare il robot UR5e con ROS2!



