"""
Test rapido per verificare che i movimenti funzionino.
Esegue un movimento molto piccolo e sicuro.
"""

import os
import sys
import time

from remote_ur_controller import RemoteURController, MoveParameters

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print(f"Connessione al robot {robot_ip}...")
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("[OK] Connesso!")
        
        # Test movimento molto piccolo: joint 0 di 0.05 rad
        print("\nTest movimento piccolo (joint 0: +0.05 rad)...")
        small_move = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
        params = MoveParameters(
            acceleration=0.5,
            velocity=0.1,
            blend_radius=0.0,
            async_move=False
        )
        
        controller.movej(small_move, params=params)
        print("[OK] Comando inviato! Attesa 3 secondi...")
        time.sleep(3)
        
        # Ritorno
        print("\nRitorno alla posizione iniziale...")
        return_move = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        controller.movej(return_move, params=params)
        time.sleep(2)
        
        print("\n[OK] Test completato!")
        
    except Exception as e:
        print(f"\n[ERR] Errore: {e}")
        return 1
    finally:
        controller.close()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

