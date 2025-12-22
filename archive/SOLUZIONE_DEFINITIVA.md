# ✅ Soluzione Definitiva - Avvia Web Interface

## 🚀 Metodo 1: Comando Unico (Copia e Incolla)

Apri PowerShell o Git Bash e incolla questo comando completo:

```bash
ssh lab@192.168.10.191 "pkill -f web_interface; sleep 2; cd ~/MekoAiAccelerator && source /opt/ros/humble/setup.bash 2>/dev/null || true && export UR_ROBOT_IP=192.168.10.194 && export WEB_HOST=0.0.0.0 && export WEB_PORT=8080 && nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 & sleep 3 && echo '✅ AVVIATA! Connettiti a: http://192.168.10.191:8080'"
```

## 🔧 Metodo 2: Manuale (Se il metodo 1 non funziona)

### Passo 1: Connettiti
```bash
ssh lab@192.168.10.191
# Password: easybot
```

### Passo 2: Esegui questi comandi uno per uno
```bash
# Ferma processi esistenti
pkill -f web_interface
sleep 2

# Vai nella directory
cd ~/MekoAiAccelerator

# Source ROS2 (se disponibile)
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Configura
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# Avvia in background
nohup python3 -m remote_ur_control.web_interface > /tmp/web_interface.log 2>&1 &

# Attendi 3 secondi
sleep 3

# Verifica che sia avviata
ps aux | grep web_interface | grep -v grep

# Se vedi un processo, è avviata!
echo "✅ Web interface avviata!"
echo "🌐 Connettiti a: http://192.168.10.191:8080"
```

## 📋 Verifica

### Vedi i log in tempo reale:
```bash
tail -f /tmp/web_interface.log
```

### Testa se risponde:
```bash
curl http://localhost:8080/api/config
```

### Se vedi JSON, funziona! ✅

## 🌐 Connettiti

Apri il browser e vai su:
```
http://192.168.10.191:8080
```

## ❌ Se non funziona

### Verifica errori:
```bash
cat /tmp/web_interface.log
```

### Verifica Flask:
```bash
python3 -c "import flask"
```

Se dà errore:
```bash
pip3 install --user flask flask-cors
```

### Verifica file:
```bash
ls -la ~/MekoAiAccelerator/remote_ur_control/web_interface.py
```

### Verifica porta:
```bash
lsof -i :8080
# Se occupata:
fuser -k 8080/tcp
```

## ✅ Dopo l'avvio

1. Apri browser: `http://192.168.10.191:8080`
2. Vedi il joystick simulato
3. Clicca e muovi per controllare il robot!

