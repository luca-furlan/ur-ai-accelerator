"""
Porta il robot in posizione iniziale/home.
"""

import os
import sys
import time

from remote_ur_controller import RemoteURController, MoveParameters

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("POSIZIONE INIZIALE ROBOT")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    # Posizione home tipica per UR (tutti i joint a 0)
    # Oppure una posizione sicura congiunta
    home_position = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]  # Posizione tipica UR
    
    print("Posizione target:")
    print(f"  Joints: {home_position}")
    print("  (0, -90°, 90°, 0, 90°, 0 gradi)\n")
    
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("[OK] Connesso")
        
        params = MoveParameters(
            acceleration=0.8,
            velocity=0.2,
            blend_radius=0.0,
            async_move=False
        )
        
        print("\nInvio comando per posizione iniziale...")
        print("[ATTENZIONE] Il robot si muoverà verso la posizione home")
        controller.movej(home_position, params=params)
        print("[OK] Comando inviato")
        
        print("\nAttesa movimento (circa 5-10 secondi)...")
        time.sleep(8)
        
        print("\n[OK] Robot dovrebbe essere in posizione iniziale")
        
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

