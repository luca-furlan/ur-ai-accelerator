#!/bin/bash
# Verifica requisiti secondo Remote Operation Guide ufficiale UR
# Fonte: Universal Robots Remote Operation Guide

set -e

echo "================================================================================"
echo "VERIFICA REQUISITI - REMOTE OPERATION GUIDE UFFICIALE"
echo "Universal Robots Remote Operation Guide"
echo "================================================================================"
echo

ROBOT_IP="192.168.10.194"

# 1. Verifica Remote Control Mode (Sezione 1.1)
echo "1. VERIFICA REMOTE CONTROL MODE (Sezione 1.1)"
echo "-----------------------------------------------"
echo "Secondo la guida: 'several functions require the robot to be in Remote Control mode'"
echo "Per abilitare: Hamburger Menu -> Settings -> System -> Remote Control -> Enable"
echo
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    # Verifica Remote Control
    sock.sendall(b"is in remote control\n")
    remote_control = sock.recv(1024).decode().strip()
    print(f"   Remote Control: {remote_control}")
    
    if "true" in remote_control.lower():
        print("   ✅ Remote Control abilitato")
    else:
        print("   ❌ Remote Control NON abilitato")
        print("   💡 Abilita: Settings -> System -> Remote Control -> Enable")
    
    sock.close()
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON
echo

# 2. Verifica stato robot (Sezione 4.1 Dashboard)
echo "2. VERIFICA STATO ROBOT (Sezione 4.1 Dashboard)"
echo "------------------------------------------------"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"robotmode\n")
    robot_mode = sock.recv(1024).decode().strip()
    print(f"   Robot Mode: {robot_mode}")
    
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    print(f"   Program State: {program_state}")
    
    sock.sendall(b"safetymode\n")
    safety_mode = sock.recv(1024).decode().strip()
    print(f"   Safety Mode: {safety_mode}")
    
    sock.close()
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON
echo

# 3. Verifica porta 50002 - External Control URCap (per ROS2)
echo "3. VERIFICA PORTA 50002 - EXTERNAL CONTROL URCAP"
echo "-------------------------------------------------"
echo "Secondo la guida: External Control URCap è necessario per ROS2 control"
echo "La porta 50002 deve essere aperta quando il programma External Control è in PLAYING"
echo
python3 << 'PYTHON'
import socket
import time
ROBOT_IP = "192.168.10.194"
MAX_RETRIES = 3

for i in range(MAX_RETRIES):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((ROBOT_IP, 50002))
        sock.close()
        
        if result == 0:
            print(f"   ✅ Porta 50002 APERTA (tentativo {i+1}/{MAX_RETRIES})")
            print("   ✅ External Control URCap attivo")
            exit(0)
        else:
            print(f"   ❌ Porta 50002 CHIUSA (tentativo {i+1}/{MAX_RETRIES})")
            time.sleep(1)
    except Exception as e:
        print(f"   ⚠️  Errore tentativo {i+1}: {e}")
        time.sleep(1)

print()
print("   ❌ Porta 50002 ancora CHIUSA dopo $MAX_RETRIES tentativi")
print()
print("   PROBLEMA: External Control URCap non configurato correttamente")
print()
print("   SOLUZIONE secondo guida:")
print("   1. Installa External Control URCap: Installation -> URCaps")
print("   2. Crea programma con nodo 'External Control'")
print("   3. Configura nodo:")
print("      - IP Host: 192.168.10.191 (IP AI Accelerator)")
print("      - Porta: 50002")
print("   4. Salva programma")
print("   5. STOP e poi PLAY di nuovo")
exit(1)
PYTHON

PORT_50002_OK=$?
echo

# 4. Verifica RTDE (Sezione 4.2)
echo "4. VERIFICA RTDE (Sezione 4.2)"
echo "-------------------------------"
echo "RTDE è disponibile di default quando il controller è in esecuzione"
echo "Porta: 30004"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ROBOT_IP, 30004))
    sock.close()
    
    if result == 0:
        print("   ✅ Porta 30004 aperta (RTDE disponibile)")
    else:
        print("   ⚠️  Porta 30004 chiusa")
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON
echo

# 5. Verifica Primary Interface (Sezione 4.3)
echo "5. VERIFICA PRIMARY INTERFACE (Sezione 4.3)"
echo "--------------------------------------------"
echo "Porta: 30001 (per comandi URScript)"
python3 << 'PYTHON'
import socket
ROBOT_IP = "192.168.10.194"
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ROBOT_IP, 30001))
    sock.close()
    
    if result == 0:
        print("   ✅ Porta 30001 aperta (Primary Interface disponibile)")
    else:
        print("   ⚠️  Porta 30001 chiusa")
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON
echo

# Riepilogo
echo "================================================================================"
echo "RIEPILOGO"
echo "================================================================================"
echo

if [ $PORT_50002_OK -eq 0 ]; then
    echo "✅ TUTTI I REQUISITI SODDISFATTI"
    echo
    echo "Puoi avviare il driver ROS2:"
    echo "  ./AVVIA_DRIVER_UFFICIALE.sh"
else
    echo "❌ PORTA 50002 CHIUSA - EXTERNAL CONTROL NON CONFIGURATO"
    echo
    echo "Segui questi passi secondo la guida ufficiale:"
    echo
    echo "1. Abilita Remote Control (se non già fatto):"
    echo "   Hamburger Menu -> Settings -> System -> Remote Control -> Enable"
    echo
    echo "2. Installa External Control URCap:"
    echo "   Installation -> URCaps -> Installa 'External Control'"
    echo
    echo "3. Crea/Modifica programma con nodo External Control:"
    echo "   - Aggiungi nodo 'External Control' al programma"
    echo "   - Configura:"
    echo "     * IP Host: 192.168.10.191"
    echo "     * Porta: 50002"
    echo "   - Salva programma"
    echo
    echo "4. STOP e poi PLAY il programma"
    echo
    echo "5. Verifica di nuovo con:"
    echo "   ./VERIFICA_REQUISITI_UFFICIALE.sh"
fi



