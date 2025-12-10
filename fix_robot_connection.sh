#!/bin/bash
# Script per risolvere problemi connessione robot e driver ROS2

echo "=========================================="
echo "FIX CONNESSIONE ROBOT E DRIVER ROS2"
echo "=========================================="
echo ""

ROBOT_IP="192.168.10.194"
AI_ACCELERATOR_IP="192.168.10.191"

# 1. Verifica connettività
echo "1. Verifica connettività robot..."
if ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile"
else
    echo "   ❌ Robot NON raggiungibile!"
    exit 1
fi

# 2. Verifica socket robot
echo ""
echo "2. Verifica socket robot..."
if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/30002" 2>/dev/null; then
    echo "   ✅ Socket 30002 aperto"
else
    echo "   ❌ Socket 30002 chiuso"
fi

if timeout 2 bash -c "echo > /dev/tcp/$ROBOT_IP/30004" 2>/dev/null; then
    echo "   ✅ Socket 30004 (RTDE) aperto"
else
    echo "   ❌ Socket 30004 chiuso"
fi

# 3. Verifica stato robot via Dashboard
echo ""
echo "3. Verifica stato robot (Dashboard)..."
python3 << 'EOF'
import socket
import sys

ROBOT_IP = "192.168.10.194"

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ROBOT_IP, 29999))
    
    # Leggi welcome message
    sock.recv(1024)
    
    # Chiedi robot mode
    sock.send(b"robotmode\n")
    mode = sock.recv(1024).decode().strip()
    print(f"   Robot mode: {mode}")
    
    # Chiedi program state
    sock.send(b"programState\n")
    state = sock.recv(1024).decode().strip()
    print(f"   Program state: {state}")
    
    # Chiedi safety mode
    sock.send(b"safetymode\n")
    safety = sock.recv(1024).decode().strip()
    print(f"   Safety mode: {safety}")
    
    sock.close()
    
    # Analizza stato
    if "RUNNING" in mode:
        print("   ✅ Robot in modalità RUNNING")
    else:
        print("   ⚠️  Robot NON in modalità RUNNING")
    
    if "PLAYING" in state:
        print("   ✅ Programma in PLAYING")
    elif "STOPPED" in state:
        print("   ⚠️  Programma STOPPED - Devi avviare External Control sul Teach Pendant!")
    else:
        print(f"   ⚠️  Stato programma: {state}")
        
except Exception as e:
    print(f"   ❌ Errore Dashboard: {e}")
    sys.exit(1)
EOF

echo ""
echo "=========================================="
echo "ISTRUZIONI PER TEACH PENDANT:"
echo "=========================================="
echo ""
echo "1. Sul Teach Pendant, vai su Program"
echo "2. Apri o crea programma con nodo External Control"
echo "3. Configura External Control:"
echo "   - IP Host: $AI_ACCELERATOR_IP"
echo "   - Porta: 50002"
echo "4. SALVA il programma"
echo "5. Premi PLAY sul Teach Pendant"
echo "6. Verifica che il programma sia in stato PLAYING"
echo ""
echo "IMPORTANTE: Il programma deve essere PLAYING prima di avviare il driver ROS2!"
echo ""
echo "=========================================="
echo "Dopo aver avviato External Control sul Teach Pendant,"
echo "riavvia il driver ROS2:"
echo "  bash avvia_driver_ur5e.sh"
echo "=========================================="





