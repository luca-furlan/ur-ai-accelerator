#!/bin/bash
# Script per verificare tutto prima di avviare driver ROS2

echo "=========================================="
echo "VERIFICA PRIMA DI AVVIARE DRIVER ROS2"
echo "=========================================="
echo ""

ROBOT_IP="192.168.10.194"
AI_ACCELERATOR_IP="192.168.10.191"

# 1. Verifica connettività
echo "1. Verifica connettività..."
if ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile"
else
    echo "   ❌ Robot NON raggiungibile!"
    exit 1
fi

# 2. Verifica stato robot
echo ""
echo "2. Verifica stato robot (Dashboard)..."
python3 << 'EOF'
import socket
import sys

ROBOT_IP = "192.168.10.194"

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)  # Welcome message
    
    # Robot mode
    sock.send(b"robotmode\n")
    mode = sock.recv(1024).decode().strip()
    print(f"   Robot mode: {mode}")
    
    # Program state
    sock.send(b"programState\n")
    state = sock.recv(1024).decode().strip()
    print(f"   Program state: {state}")
    
    # Safety mode
    sock.send(b"safetymode\n")
    safety = sock.recv(1024).decode().strip()
    print(f"   Safety mode: {safety}")
    
    sock.close()
    
    # Verifica condizioni
    ok = True
    
    if "RUNNING" not in mode:
        print("   ❌ Robot NON in modalità RUNNING!")
        print("      Sul Teach Pendant: Premi il pulsante START (verde)")
        ok = False
    else:
        print("   ✅ Robot in modalità RUNNING")
    
    if "PLAYING" not in state:
        print("   ❌ Programma NON in stato PLAYING!")
        print("      Sul Teach Pendant:")
        print("      1. Vai su Program")
        print("      2. Apri programma con External Control")
        print("      3. Configura External Control:")
        print("         - IP Host: 192.168.10.191")
        print("         - Porta: 50002")
        print("      4. Premi PLAY")
        ok = False
    else:
        print("   ✅ Programma in stato PLAYING")
    
    if "NORMAL" not in safety:
        print(f"   ⚠️  Safety mode: {safety}")
    else:
        print("   ✅ Safety mode: NORMAL")
    
    if not ok:
        print("")
        print("   ⚠️  CORREGGI I PROBLEMI SOPRA PRIMA DI AVVIARE IL DRIVER!")
        sys.exit(1)
    else:
        print("")
        print("   ✅ TUTTO OK! Puoi avviare il driver ROS2")
        
except Exception as e:
    print(f"   ❌ Errore: {e}")
    sys.exit(1)
EOF

EXIT_CODE=$?

echo ""
echo "=========================================="

if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ VERIFICA COMPLETATA - TUTTO OK!"
    echo ""
    echo "Puoi avviare il driver con:"
    echo "  bash avvia_driver_ur5e_fixed.sh"
else
    echo "❌ VERIFICA FALLITA - CORREGGI I PROBLEMI!"
    echo ""
    echo "Dopo aver corretto, riprova:"
    echo "  bash verifica_prima_di_avviare.sh"
fi

echo "=========================================="











