"""
Avvia il programma sul robot e testa i movimenti.
"""

import os
import socket
import sys
import time

from remote_ur_controller import RemoteURController, MoveParameters

def start_program(robot_ip: str):
    """Avvia il programma sul robot."""
    print("[STEP 1] Avvio programma...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((robot_ip, 29999))
    sock.recv(1024)  # Welcome
    
    # Stop se necessario
    sock.sendall(b"stop\n")
    time.sleep(0.3)
    sock.recv(1024)
    
    # Play
    sock.sendall(b"play\n")
    time.sleep(0.5)
    response = sock.recv(1024).decode('utf-8').strip()
    print(f"  Risposta: {response}")
    
    sock.close()
    
    if "Starting program" in response or "Playing" in response:
        print("  [OK] Programma avviato!")
        time.sleep(2)
        return True
    else:
        print(f"  [ATTENZIONE] Risposta: {response}")
        return False

def test_movement(robot_ip: str):
    """Test movimento dopo avvio programma."""
    print("\n[STEP 2] Test movimento...")
    
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("  [OK] Connesso!")
        
        # Movimento visibile
        print("\n  Movimento Joint 0: +0.25 rad (~14 gradi)")
        movement = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0]
        params = MoveParameters(acceleration=0.8, velocity=0.2)
        
        controller.movej(movement, params=params)
        print("  [OK] Comando inviato!")
        print("  [ATTENZIONE] Il robot dovrebbe muoversi ORA!")
        print("  Attesa 3 secondi...")
        time.sleep(3)
        
        # Ritorno
        print("\n  Ritorno alla posizione iniziale...")
        controller.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], params=params)
        print("  [OK] Comando inviato!")
        time.sleep(3)
        
        print("\n  [OK] Test completato!")
        return True
        
    except Exception as e:
        print(f"  [ERR] Errore: {e}")
        return False
    finally:
        controller.close()

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("AVVIO PROGRAMMA E TEST MOVIMENTO")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    # Avvia programma
    if not start_program(robot_ip):
        print("\n[ATTENZIONE] Impossibile avviare programma automaticamente.")
        print("Prova a premere PLAY manualmente sul teach pendant.")
        print("Poi esegui di nuovo questo script.\n")
    
    # Test movimento
    success = test_movement(robot_ip)
    
    print("\n" + "=" * 60)
    if success:
        print("[OK] Processo completato!")
        print("\nIl robot si e mosso?")
        print("  - Se SI: tutto funziona! Puoi usare i comandi normalmente.")
        print("  - Se NO: verifica sul teach pendant che il programma sia PLAYING")
    else:
        print("[ATTENZIONE] Verifica manualmente sul teach pendant")
    print("=" * 60)
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())

