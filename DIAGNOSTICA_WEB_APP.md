# 🔧 Diagnostica Web App Non Funzionante

## Problema: Robot acceso ma web app non funziona

### Soluzione Rapida

**Connettiti all'AI Accelerator ed esegui:**

```bash
ssh lab@192.168.10.191
cd ~/MekoAiAccelerator
bash fix_web_interface.sh
```

Lo script:
1. ✅ Ferma processi esistenti
2. ✅ Verifica file e dipendenze
3. ✅ Verifica connessione robot
4. ✅ Libera porta 8080 se occupata
5. ✅ Riavvia web interface
6. ✅ Mostra URL per connettersi

### Oppure Manualmente

```bash
ssh lab@192.168.10.191

# 1. Ferma processi esistenti
pkill -f web_interface

# 2. Vai nella directory
cd ~/MekoAiAccelerator

# 3. Source ROS2 (se disponibile)
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# 4. Configura
export UR_ROBOT_IP=192.168.10.194
export WEB_HOST=0.0.0.0
export WEB_PORT=8080

# 5. Avvia
python3 -m remote_ur_control.web_interface
```

### Verifica Errori

Se non funziona, controlla i log:

```bash
# Vedi log in tempo reale
tail -f /tmp/web_interface.log

# Oppure se avviato in foreground, vedi errori direttamente
```

### Problemi Comuni

#### 1. Porta 8080 occupata

```bash
# Trova processo che usa porta 8080
lsof -i :8080
# o
fuser 8080/tcp

# Fermalo
fuser -k 8080/tcp
```

#### 2. Flask non installato

```bash
pip3 install --user flask flask-cors
```

#### 3. File mancanti

```bash
# Verifica file
ls -la ~/MekoAiAccelerator/remote_ur_control/web_interface.py

# Se manca, esegui deploy
bash deploy_to_ai_accelerator_complete.sh
```

#### 4. Robot non raggiungibile

```bash
# Verifica connessione
ping 192.168.10.194

# Verifica porta robot
timeout 2 bash -c "</dev/tcp/192.168.10.194/30002" && echo "OK" || echo "FAIL"
```

#### 5. Errori Python

```bash
# Testa import
python3 -c "from remote_ur_control.web_interface import app"
```

### Test Connessione

Dopo aver avviato, testa se risponde:

```bash
# Dall'AI Accelerator
curl http://localhost:8080/api/config

# Dovresti vedere JSON con configurazione
```

### URL Finale

Una volta avviata correttamente, connettiti a:

```
http://192.168.10.191:8080
```

