# 🎮 Guida Controllo Robot con Joystick e MuJoCo

## 🎮 OPZIONE 1: Joystick Fisico

### Installazione

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator

# Installa pygame per supporto joystick
pip3 install --user pygame
```

### Uso

```bash
python3 controllo_joystick_fisico.py
```

**Controlli:**
- **Left Stick X/Y**: Movimento TCP X/Y
- **Right Stick Y**: Movimento TCP Z (su/giù)
- **Right Stick X**: Rotazione TCP RX
- **START**: Esci

**Funziona con:**
- Joystick USB standard
- Gamepad Xbox/PlayStation
- Qualsiasi dispositivo riconosciuto come joystick

## 🖥️ OPZIONE 2: Joystick Virtuale (Web Interface)

### Uso

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
bash avvia_web_interface_ur5e.sh
```

**Browser:** `http://192.168.10.191:8080`

**Controlli:**
- Joystick virtuale sullo schermo
- Muovi con mouse/touch
- Controllo fluido del robot

## 🤖 OPZIONE 3: MuJoCo - Simulazione e Controllo Reale

### Uso

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
python3 controllo_mujoco_robot.py
```

**Come funziona:**
1. MuJoCo mostra simulazione robot UR5e
2. Muovi il robot in simulazione con il mouse
3. Premi **SPACE** per inviare posizione al robot reale
4. Il robot reale si muove alla posizione simulata

**Controlli:**
- **Mouse**: Muovi robot in simulazione
- **SPACE**: Invia posizione al robot reale
- **ESC**: Esci

## 📋 Confronto Metodi

| Metodo | Tipo | Vantaggi | Quando Usare |
|--------|------|----------|--------------|
| **Joystick Fisico** | Hardware | Controllo preciso, feedback tattile | Controllo manuale preciso |
| **Web Interface** | Browser | Accesso remoto, facile da usare | Controllo da qualsiasi dispositivo |
| **MuJoCo** | Simulazione | Test prima di eseguire, visualizzazione 3D | Pianificazione traiettorie |

## 🚀 Quick Start

### Per controllo immediato:
```bash
# Web Interface (più facile)
bash avvia_web_interface_ur5e.sh
# Apri browser: http://192.168.10.191:8080
```

### Per controllo con joystick fisico:
```bash
# Installa pygame
pip3 install --user pygame

# Collega joystick USB
python3 controllo_joystick_fisico.py
```

### Per simulazione + controllo reale:
```bash
python3 controllo_mujoco_robot.py
# Muovi robot in simulazione, premi SPACE per inviare
```

## ⚠️ SICUREZZA

- **Sempre** verifica che l'area intorno al robot sia libera
- **Inizia** con movimenti piccoli
- **Usa** velocità basse per i primi test
- **Tieni** sempre il pulsante STOP del Teach Pendant a portata di mano

## ✅ TUTTO PRONTO!

Hai 3 modi per controllare il robot:
1. ✅ **Joystick fisico** - Controllo preciso
2. ✅ **Web Interface** - Controllo remoto facile
3. ✅ **MuJoCo** - Simulazione + controllo reale

**Scegli il metodo che preferisci!** 🚀





