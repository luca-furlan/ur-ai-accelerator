#!/bin/bash
# Setup secondo guida pratica GitHub Gist
# Fonte: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db
# AUTORE: Shawn Armstrong

set -e

echo "================================================================================"
echo "SETUP UR5e CON ROS2 - GUIDA PRATICA"
echo "Fonte: https://gist.github.com/Shawn-Armstrong/bdbcd51e0d60a0a4e4b60d15c635d3db"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

# PASSO 1: Scarica URCap External Control
echo "PASSO 1: SCARICA URCAP EXTERNAL CONTROL"
echo "----------------------------------------"
echo "Scarica: externalcontrol-1.0.5.urcap"
echo

URCAP_FILE="externalcontrol-1.0.5.urcap"
URCAP_URL="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/${URCAP_FILE}"

if [ -f "$URCAP_FILE" ]; then
    echo "✅ File URCap già presente: $URCAP_FILE"
else
    echo "📥 Scaricamento URCap..."
    if command -v wget >/dev/null 2>&1; then
        wget "$URCAP_URL" || curl -L -o "$URCAP_FILE" "$URCAP_URL"
    elif command -v curl >/dev/null 2>&1; then
        curl -L -o "$URCAP_FILE" "$URCAP_URL"
    else
        echo "❌ wget o curl non disponibili. Installa uno dei due."
        exit 1
    fi
    
    if [ -f "$URCAP_FILE" ]; then
        echo "✅ URCap scaricato: $URCAP_FILE"
    else
        echo "❌ Errore scaricamento URCap"
        exit 1
    fi
fi
echo

# PASSO 2: Istruzioni per installazione URCap sul Teach Pendant
echo "================================================================================"
echo "PASSO 2: INSTALLA URCAP SUL ROBOT (TEACH PENDANT)"
echo "================================================================================"
echo
echo "Il file URCap è stato scaricato: $URCAP_FILE"
echo
echo "SEGUI QUESTI PASSI SUL TEACH PENDANT:"
echo
echo "1. Copia il file '$URCAP_FILE' su una chiavetta USB"
echo "   (Il file si trova nella directory corrente)"
echo
echo "2. Inserisci la chiavetta USB nel Teach Pendant"
echo
echo "3. Sul Teach Pendant:"
echo "   - Vai su: Installation → URCaps"
echo "   - Premi il pulsante '+' (Aggiungi)"
echo "   - Seleziona il file '$URCAP_FILE' dalla USB"
echo "   - Premi 'Apri'"
echo "   - Riavvia il robot quando richiesto"
echo
echo "================================================================================"
read -p "Premi INVIO quando hai installato l'URCap e riavviato il robot..."
echo

# PASSO 3: Istruzioni per configurare programma sul Teach Pendant
echo "================================================================================"
echo "PASSO 3: CONFIGURA PROGRAMMA SUL TEACH PENDANT"
echo "================================================================================"
echo
echo "SEGUI QUESTI PASSI SUL TEACH PENDANT:"
echo
echo "1. Crea un nuovo programma (o apri 'remote_control.urp')"
echo
echo "2. Aggiungi nodo 'External Control' al programma:"
echo "   - Vai su: Structure → URCaps → External Control"
echo "   - Trascina il nodo nel programma"
echo
echo "3. Configura il nodo External Control:"
echo "   - Clicca sul nodo External Control"
echo "   - IP Host: 192.168.10.191 (IP AI Accelerator)"
echo "   - Porta: 50002 (default)"
echo
echo "4. Aggiungi script iniziale (opzionale ma consigliato):"
echo "   - Prima del nodo External Control, aggiungi uno script"
echo "   - Inserisci: movej([0, 0, 0, 0, 0, 0], a=1.0, v=1.0)"
echo "   - Questo porta il robot in posizione iniziale"
echo
echo "5. Salva il programma (es: 'ros_control.urp')"
echo
echo "================================================================================"
read -p "Premi INVIO quando hai configurato il programma sul Teach Pendant..."
echo

# PASSO 4: Verifica configurazione
echo "================================================================================"
echo "PASSO 4: VERIFICA CONFIGURAZIONE"
echo "================================================================================"
echo

# Verifica Remote Control
echo "Verifica Remote Control..."
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"is in remote control\n")
    remote_control = sock.recv(1024).decode().strip()
    print(f"   Remote Control: {remote_control}")
    
    if "true" in remote_control.lower():
        print("   ✅ Remote Control abilitato")
    else:
        print("   ❌ Remote Control NON abilitato")
        print("   💡 Abilita: Settings -> System -> Remote Control -> Enable")
        exit(1)
    
    sock.close()
except Exception as e:
    print(f"   ❌ Errore: {e}")
    exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "Abilita Remote Control sul Teach Pendant e riprova!"
    exit 1
fi
echo

# Verifica porta 50002
echo "Verifica porta 50002 (External Control)..."
python3 << 'PYTHON'
import socket
import time
ROBOT_IP = "192.168.10.194"
MAX_RETRIES = 5

print("   ⏳ Attendo che il programma sia in PLAYING...")
for i in range(MAX_RETRIES):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((ROBOT_IP, 50002))
        sock.close()
        
        if result == 0:
            print(f"   ✅ Porta 50002 APERTA! (tentativo {i+1}/{MAX_RETRIES})")
            print("   ✅ External Control configurato correttamente")
            exit(0)
        else:
            print(f"   ⏳ Porta ancora chiusa... (tentativo {i+1}/{MAX_RETRIES})")
            print("   💡 Assicurati che il programma sia in PLAYING sul Teach Pendant")
            time.sleep(2)
    except Exception as e:
        print(f"   ⚠️  Errore tentativo {i+1}: {e}")
        time.sleep(2)

print()
print("   ❌ Porta 50002 ancora CHIUSA dopo $MAX_RETRIES tentativi")
print()
print("   VERIFICA:")
print("   1. Il programma è in PLAYING sul Teach Pendant?")
print("   2. Il nodo External Control è presente nel programma?")
print("   3. L'IP Host è configurato come 192.168.10.191?")
print("   4. La porta è configurata come 50002?")
exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "Configura il programma sul Teach Pendant e mettilo in PLAYING!"
    exit 1
fi
echo

# PASSO 5: Avvia driver ROS2
echo "================================================================================"
echo "PASSO 5: AVVIA DRIVER ROS2"
echo "================================================================================"
echo

source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✅ ROS2 workspace configurato"
fi

echo "Avvio driver ROS2..."
echo "Comando: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${ROBOT_TYPE} robot_ip:=${ROBOT_IP} launch_rviz:=false"
echo
echo "⚠️  NOTA: Secondo la guida, launch_rviz:=false evita conflitti di rete"
echo

# Ferma eventuali driver già in esecuzione
pkill -f "ur_control.launch.py" 2>/dev/null || true
sleep 2

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=${ROBOT_TYPE} \
    robot_ip:=${ROBOT_IP} \
    launch_rviz:=false









