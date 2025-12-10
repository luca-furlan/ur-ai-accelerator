#!/bin/bash
# Script per fixare il controllo robot - usa fallback socket se ROS2 non disponibile

set -e

echo "================================================================================"
echo "FIX CONTROLLO ROBOT - USA SOCKET DIRETTO"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"

# 1. Verifica stato robot
echo "1. Verifica stato robot..."
python3 << 'PYTHON'
import socket
import sys

ROBOT_IP = "192.168.10.194"

try:
    # Dashboard Server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)  # Welcome message
    
    sock.sendall(b"robotmode\n")
    robot_mode = sock.recv(1024).decode().strip()
    print(f"   Robot Mode: {robot_mode}")
    
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    print(f"   Program State: {program_state}")
    
    sock.close()
    
    if "PLAYING" not in program_state:
        print()
        print("   ❌ PROBLEMA: Programma NON in PLAYING!")
        print("   💡 SOLUZIONE: Vai sul teach pendant e:")
        print("      1. Avvia il programma")
        print("      2. Mettilo in PLAYING")
        print("      3. Verifica che Remote Control sia attivo")
        sys.exit(1)
    else:
        print("   ✅ Programma in PLAYING - OK!")
        
except Exception as e:
    print(f"   ❌ Errore connessione: {e}")
    sys.exit(1)
PYTHON

if [ $? -ne 0 ]; then
    echo
    echo "================================================================================"
    echo "❌ ERRORE: Il programma NON è in PLAYING!"
    echo "================================================================================"
    echo
    echo "Vai sul teach pendant del robot e:"
    echo "1. Avvia il programma (es. 'remote_control.urp')"
    echo "2. Premi PLAY per metterlo in PLAYING"
    echo "3. Verifica che 'Remote Control' sia attivo"
    echo
    echo "Poi riprova a controllare il robot dalla web interface."
    exit 1
fi

echo

# 2. Test invio comando diretto
echo "2. Test invio comando diretto..."
python3 << 'PYTHON'
import socket
import time

ROBOT_IP = "192.168.10.194"

try:
    # URScript Port (30002)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((ROBOT_IP, 30002))
    
    # Piccolo movimento joint 1 (0.05 rad/s per 0.5s)
    script = "speedj([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\n"
    sock.sendall(script.encode('utf-8'))
    print("   ✅ Comando inviato via socket")
    print("   💡 Il robot dovrebbe muoversi leggermente (joint 1)")
    
    time.sleep(0.5)
    
    # Stop
    script = "stopj(1.0)\n"
    sock.sendall(script.encode('utf-8'))
    
    sock.close()
    print("   ✅ Test completato")
    
except Exception as e:
    print(f"   ❌ Errore: {e}")
    print("   💡 Verifica che il programma sia in PLAYING")
PYTHON

echo
echo "================================================================================"
echo "DIAGNOSI"
echo "================================================================================"
echo
echo "Se il robot si è mosso durante il test:"
echo "   ✅ Il controllo socket funziona!"
echo "   ✅ Il problema è nella web interface"
echo
echo "Se il robot NON si è mosso:"
echo "   ❌ Il programma NON è in PLAYING"
echo "   ❌ O c'è un problema di connessione"
echo
echo "================================================================================"
echo "SOLUZIONE PER WEB INTERFACE"
echo "================================================================================"
echo
echo "La web interface usa ROS2 per default, ma se ROS2 non funziona,"
echo "usa il fallback socket. Per forzare l'uso del socket:"
echo
echo "1. Verifica che il programma sia in PLAYING sul teach pendant"
echo "2. La web interface userà automaticamente il socket se ROS2 fallisce"
echo "3. Se vuoi usare solo socket, modifica web_interface.py per saltare ROS2"
echo




