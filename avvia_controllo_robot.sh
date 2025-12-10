#!/bin/bash
# Script per avviare controllo robot - mostra tutte le opzioni

echo "=========================================="
echo "CONTROLLO ROBOT UR5e - SCEGLI METODO"
echo "=========================================="
echo ""
echo "Metodi disponibili:"
echo ""
echo "1. 🖥️  Web Interface (Joystick Virtuale)"
echo "   - Controllo da browser"
echo "   - Joystick virtuale sullo schermo"
echo "   - Accesso remoto"
echo ""
echo "2. 🎮 Joystick Fisico"
echo "   - Controllo con joystick USB/gamepad"
echo "   - Feedback tattile"
echo "   - Controllo preciso"
echo ""
echo "3. 🤖 MuJoCo (Simulazione + Controllo Reale)"
echo "   - Simula robot in 3D"
echo "   - Muovi in simulazione, invia al robot reale"
echo "   - Test traiettorie prima di eseguire"
echo ""
echo "4. 📊 RTDE (Lettura Dati)"
echo "   - Leggi posizioni joints"
echo "   - Monitora stato robot"
echo ""
echo "=========================================="
echo ""

read -p "Scegli metodo (1-4): " scelta

case $scelta in
    1)
        echo ""
        echo "Avvio Web Interface..."
        echo "Apri browser: http://192.168.10.191:8080"
        echo ""
        bash avvia_web_interface_ur5e.sh
        ;;
    2)
        echo ""
        echo "Avvio controllo joystick fisico..."
        echo "Collega joystick USB e premi ENTER"
        read
        python3 controllo_joystick_fisico.py
        ;;
    3)
        echo ""
        echo "Avvio MuJoCo..."
        echo "Muovi robot in simulazione, premi SPACE per inviare al robot reale"
        echo ""
        python3 controllo_mujoco_robot.py
        ;;
    4)
        echo ""
        echo "Lettura dati RTDE..."
        python3 << 'EOF'
import rtde.rtde as rtde
import math

ROBOT_IP = "192.168.10.194"
con = rtde.RTDE(ROBOT_IP, 30004)
con.connect()
con.send_output_setup(["actual_q", "actual_TCP_pose"], [], frequency=10)
con.send_start()

print("Lettura dati (CTRL+C per fermare)...")
print()

try:
    while True:
        state = con.receive()
        if state:
            joints = [round(math.degrees(j), 1) for j in state.actual_q]
            tcp = state.actual_TCP_pose
            print(f"\rJoints: {joints}° | TCP: [{tcp[0]*1000:.1f}, {tcp[1]*1000:.1f}, {tcp[2]*1000:.1f}] mm", end="", flush=True)
        import time
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n\nStop")
finally:
    con.disconnect()
EOF
        ;;
    *)
        echo "Scelta non valida"
        exit 1
        ;;
esac
