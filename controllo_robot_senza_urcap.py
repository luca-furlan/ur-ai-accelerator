#!/usr/bin/env python3
"""
Controllo robot UR5e SENZA External Control URCap
Usa Primary Interface (porta 30001) - URScript diretto
"""

import socket
import time
import sys

ROBOT_IP = "192.168.10.194"
PRIMARY_PORT = 30001

def send_urscript(script):
    """Invia comando URScript al robot via Primary Interface"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ROBOT_IP, PRIMARY_PORT))
        sock.send(script.encode() + b"\n")
        time.sleep(0.1)
        sock.close()
        return True
    except Exception as e:
        print(f"❌ Errore invio comando: {e}")
        return False

def test_connection():
    """Test connessione Primary Interface"""
    print("=" * 60)
    print("TEST CONNESSIONE PRIMARY INTERFACE")
    print("=" * 60)
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Porta: {PRIMARY_PORT}")
    print()
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((ROBOT_IP, PRIMARY_PORT))
        sock.close()
        print("✅ Primary Interface raggiungibile!")
        return True
    except Exception as e:
        print(f"❌ Primary Interface NON raggiungibile: {e}")
        return False

def move_small_test():
    """Movimento piccolo e sicuro per test"""
    print()
    print("=" * 60)
    print("TEST MOVIMENTO ROBOT")
    print("=" * 60)
    print("⚠️  ATTENZIONE: Il robot si muoverà!")
    print("   Movimento: +10mm in Z, poi torna indietro")
    print()
    
    risposta = input("Continuare? (s/n): ")
    if risposta.lower() != 's':
        print("❌ Annullato")
        return False
    
    script = """
def test_move():
    # Leggi posizione attuale TCP
    current_pose = get_actual_tcp_pose()
    
    # Crea nuova posizione (10mm più in alto)
    target_pose = current_pose
    target_pose[2] = target_pose[2] + 0.01  # +10mm in Z
    
    # Muovi lentamente e in sicurezza
    movel(target_pose, a=0.1, v=0.05)
    
    # Aspetta
    sleep(1)
    
    # Torna alla posizione originale
    movel(current_pose, a=0.1, v=0.05)
end

test_move()
"""
    
    print("Invio comando movimento...")
    if send_urscript(script):
        print("✅ Comando inviato!")
        print("Il robot dovrebbe muoversi di 10mm in Z e tornare indietro.")
        return True
    else:
        print("❌ Errore invio comando")
        return False

def read_joint_positions():
    """Leggi posizioni joints attuali"""
    print()
    print("=" * 60)
    print("LETTURA POSIZIONI JOINTS")
    print("=" * 60)
    
    script = """
def read_joints():
    joints = get_actual_joint_positions()
    textmsg("Joints: ", joints[0], ", ", joints[1], ", ", joints[2], ", ", joints[3], ", ", joints[4], ", ", joints[5])
end

read_joints()
"""
    
    print("Invio comando lettura...")
    if send_urscript(script):
        print("✅ Comando inviato!")
        print("Controlla i messaggi sul Teach Pendant per vedere le posizioni joints.")
        return True
    else:
        print("❌ Errore invio comando")
        return False

def main():
    print("=" * 60)
    print("CONTROLLO ROBOT UR5e - SENZA EXTERNAL CONTROL URCAP")
    print("Usa Primary Interface (porta 30001)")
    print("=" * 60)
    print()
    
    # Test connessione
    if not test_connection():
        print()
        print("❌ Impossibile connettersi al robot!")
        print("Verifica:")
        print("  1. Robot acceso e raggiungibile")
        print("  2. Robot in modalità RUNNING")
        print("  3. Connessione di rete OK")
        return
    
    print()
    print("Cosa vuoi fare?")
    print("  1) Test movimento piccolo (10mm in Z)")
    print("  2) Leggi posizioni joints")
    print("  3) Esci")
    print()
    
    scelta = input("Scelta (1-3): ")
    
    if scelta == "1":
        move_small_test()
    elif scelta == "2":
        read_joint_positions()
    else:
        print("Uscita")
    
    print()
    print("=" * 60)
    print("NOTA: Questo metodo funziona ma è limitato.")
    print("Per controllo completo, installa External Control URCap.")
    print("=" * 60)

if __name__ == "__main__":
    main()





