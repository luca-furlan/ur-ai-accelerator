"""
Test SICURO con movimenti molto piccoli e controllati.
Usa solo movimenti minimi per verificare la connessione.
"""

import os
import sys
import time

from remote_ur_controller import RemoteURController, MoveParameters

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("TEST SICURO - MOVIMENTI MINIMI")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    print("[ATTENZIONE] Questo test usa movimenti MOLTO piccoli (0.05 rad = ~3 gradi)")
    print("Solo per verificare che la connessione funzioni.\n")
    
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("[OK] Connesso\n")
        
        # Movimento MOLTO piccolo e sicuro
        print("[TEST] Movimento minimo - Joint 0: +0.05 rad (~3 gradi)")
        print("  Questo è un movimento molto piccolo e sicuro\n")
        
        movement = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]  # Solo 0.05 rad = ~3 gradi
        params = MoveParameters(
            acceleration=0.3,  # Molto bassa
            velocity=0.05,      # Molto bassa
            blend_radius=0.0,
            async_move=False
        )
        
        print(f"  Parametri: accel={params.acceleration}, vel={params.velocity}")
        print(f"  Target: {movement}")
        print("\n  Invio comando...")
        
        controller.movej(movement, params=params)
        print("  [OK] Comando inviato")
        print("  Attesa 2 secondi per movimento lento...")
        time.sleep(2)
        
        # Ritorno molto lento
        print("\n  Ritorno alla posizione iniziale (molto lento)...")
        controller.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], params=params)
        print("  [OK] Comando inviato")
        print("  Attesa 2 secondi...")
        time.sleep(2)
        
        print("\n[OK] Test completato con sicurezza")
        print("\nSe il robot si è mosso leggermente, la connessione funziona.")
        print("Per movimenti più grandi, aumenta gradualmente i valori.")
        
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

