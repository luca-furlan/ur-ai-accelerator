"""
Test completo di tutti i comandi di movimento disponibili.
"""

import os
import sys
import time

from remote_ur_controller import RemoteURController, MoveParameters

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("TEST COMPLETO COMANDI MOVIMENTO")
    print("=" * 60)
    
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("[OK] Connesso al robot\n")
        
        # Test 1: MoveJ
        print("[TEST 1] MoveJ - movimento piccolo...")
        small_move = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
        params = MoveParameters(acceleration=0.5, velocity=0.1)
        controller.movej(small_move, params=params)
        print("  Comando inviato, attesa 2s...")
        time.sleep(2)
        
        # Ritorno
        controller.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], params=params)
        time.sleep(2)
        print("[OK] MoveJ testato\n")
        
        # Test 2: SpeedJ
        print("[TEST 2] SpeedJ - velocita joint...")
        speeds = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
        controller.speedj(speeds, duration=1.0, acceleration=0.3)
        print("  Comando inviato, attesa 1.5s...")
        time.sleep(1.5)
        controller.stop()
        print("[OK] SpeedJ testato\n")
        
        # Test 3: SpeedL
        print("[TEST 3] SpeedL - velocita cartesiana...")
        cart_speeds = [0.0, 0.01, 0.0, 0.0, 0.0, 0.0]  # 10mm/s in Y
        controller.speedl(cart_speeds, duration=1.0, acceleration=0.2)
        print("  Comando inviato, attesa 1.5s...")
        time.sleep(1.5)
        controller.stop()
        print("[OK] SpeedL testato\n")
        
        print("=" * 60)
        print("[OK] Tutti i test completati con successo!")
        print("=" * 60)
        
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

