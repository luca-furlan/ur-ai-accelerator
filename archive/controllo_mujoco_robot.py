#!/usr/bin/env python3
"""
Controllo robot UR5e reale tramite MuJoCo
MuJoCo simula e invia comandi al robot reale
"""

import mujoco
import mujoco.viewer
import numpy as np
import socket
import time
import sys
import os

ROBOT_IP = "192.168.10.194"
PRIMARY_PORT = 30001

# Path modello UR5e
MUJOCO_MODEL_PATH = os.path.expanduser("~/mujoco_menagerie/universal_robots_ur5e/scene.xml")

def send_urscript(script):
    """Invia comando URScript al robot"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        sock.connect((ROBOT_IP, PRIMARY_PORT))
        sock.send(script.encode() + b"\n")
        time.sleep(0.05)
        sock.close()
        return True
    except Exception as e:
        print(f"Errore invio: {e}")
        return False

def joints_to_urscript(joints):
    """Converte posizioni joints in comando URScript movej"""
    script = f"""
def move_to_joints():
    target_joints = [{joints[0]}, {joints[1]}, {joints[2]}, {joints[3]}, {joints[4]}, {joints[5]}]
    movej(target_joints, a=0.5, v=0.3)
end

move_to_joints()
"""
    return script

def main():
    print("=" * 60)
    print("CONTROLLO ROBOT UR5e VIA MUJOCO")
    print("=" * 60)
    print()
    
    # Verifica modello
    if not os.path.exists(MUJOCO_MODEL_PATH):
        print(f"❌ Modello MuJoCo non trovato: {MUJOCO_MODEL_PATH}")
        print("   Installa: git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie")
        return
    
    print(f"Caricamento modello: {MUJOCO_MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(MUJOCO_MODEL_PATH)
    data = mujoco.MjData(model)
    
    print("✅ Modello caricato")
    print()
    print("Controlli:")
    print("  Muovi il robot in MuJoCo con il mouse")
    print("  Premi SPACE per inviare posizione al robot reale")
    print("  Premi ESC per uscire")
    print()
    print("⚠️  ATTENZIONE: Il robot reale si muoverà!")
    print()
    
    input("Premi ENTER per iniziare...")
    
    print("Avvio viewer MuJoCo...")
    print("Muovi il robot e premi SPACE per inviare comando")
    print()
    
    last_sent_joints = None
    
    def key_callback(keycode):
        """Callback per tasti"""
        nonlocal last_sent_joints
        
        if keycode == 32:  # SPACE
            # Invia posizione corrente al robot
            current_joints = data.qpos[:6].copy()
            
            # Converti in comando URScript
            script = joints_to_urscript(current_joints)
            
            if send_urscript(script):
                print(f"✅ Comando inviato! Joints: {np.degrees(current_joints)}")
                last_sent_joints = current_joints.copy()
            else:
                print("❌ Errore invio comando")
    
    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            print("✅ Viewer avviato")
            print("Muovi il robot e premi SPACE per inviare")
            print()
            
            while viewer.is_running():
                # Step simulazione
                mujoco.mj_step(model, data)
                
                # Aggiorna viewer
                viewer.sync()
                
                time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nInterrotto")
    finally:
        print("✅ Chiusura viewer")
        # Stop robot
        script = joints_to_urscript([0, 0, 0, 0, 0, 0])
        send_urscript(script)

if __name__ == "__main__":
    main()











