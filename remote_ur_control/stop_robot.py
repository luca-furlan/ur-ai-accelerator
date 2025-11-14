"""
Script per fermare immediatamente il robot.
"""

import os
import sys

from remote_ur_controller import RemoteURController

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("FERMATA ROBOT")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    controller = RemoteURController(robot_ip=robot_ip, port=30002)
    
    try:
        controller.connect()
        print("[OK] Connesso")
        
        print("\nInvio comando STOP...")
        controller.stop()
        print("[OK] Comando STOP inviato!")
        
        # Invia stop anche via socket diretto per sicurezza
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((robot_ip, 30002))
        stop_script = "def emergency_stop():\n  stopl(1.5)\nend\nemergency_stop()\n"
        sock.sendall(stop_script.encode('utf-8'))
        sock.close()
        print("[OK] Comando STOP aggiuntivo inviato")
        
        print("\n[OK] Robot dovrebbe essere fermo ora")
        
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

