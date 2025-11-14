"""
Test con movimento più visibile per verificare che il robot si muova effettivamente.
"""

import os
import sys
import time

from remote_ur_controller import RemoteURController, MoveParameters

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("TEST MOVIMENTO VISIBILE")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("[OK] Connesso!\n")
        
        # Movimento più grande e visibile: joint 0 di 0.3 rad (~17 gradi)
        print("[TEST] Movimento visibile - Joint 0: +0.3 rad (~17 gradi)")
        print("  [ATTENZIONE] Il robot dovrebbe muoversi chiaramente!\n")
        
        movement = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
        params = MoveParameters(
            acceleration=0.8,
            velocity=0.2,
            blend_radius=0.0,
            async_move=False
        )
        
        print(f"  Invio comando movej: {movement}")
        controller.movej(movement, params=params)
        print("  [OK] Comando inviato!")
        print("  Attesa 4 secondi per osservare il movimento...\n")
        time.sleep(4)
        
        # Ritorno alla posizione iniziale
        print("[TEST] Ritorno alla posizione iniziale...")
        return_move = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print(f"  Invio comando movej: {return_move}")
        controller.movej(return_move, params=params)
        print("  [OK] Comando inviato!")
        print("  Attesa 4 secondi...\n")
        time.sleep(4)
        
        print("=" * 60)
        print("[OK] Test completato!")
        print("=" * 60)
        print("\nIl robot si e mosso?")
        print("  - Se SI: tutto funziona correttamente!")
        print("  - Se NO: verifica sul teach pendant:")
        print("    1. Programma in PLAYING?")
        print("    2. Modalita REMOTE attiva?")
        print("    3. Errori o warning visibili?")
        
    except Exception as e:
        print(f"\n[ERR] Errore: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        controller.close()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

