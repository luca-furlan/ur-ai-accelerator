#!/usr/bin/env python3
"""
Script per verificare VNC e Power On sul robot UR5e.
"""

import socket
import sys
import time
import subprocess

ROBOT_IP = "192.168.10.194"
DASHBOARD_PORT = 29999
VNC_PORT = 5900

def check_port(ip: str, port: int, timeout: float = 3.0) -> bool:
    """Verifica se una porta è aperta."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except Exception as e:
        print(f"  [ERR] Errore verifica porta: {e}")
        return False

def send_dashboard_command(robot_ip: str, command: str) -> tuple[bool, str]:
    """Invia comando al Dashboard Server e ritorna (success, response)."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((robot_ip, DASHBOARD_PORT))
        
        # Leggi welcome message
        welcome = sock.recv(1024)
        print(f"  [WELCOME] {welcome.decode('utf-8').strip()[:50]}")
        
        # Invia comando
        sock.sendall((command + "\n").encode('utf-8'))
        time.sleep(0.3)
        
        # Leggi risposta
        response = sock.recv(1024).decode('utf-8').strip()
        sock.close()
        
        return True, response
    except socket.timeout:
        return False, "Timeout connessione"
    except ConnectionRefusedError:
        return False, "Connessione rifiutata - robot spento o Dashboard Server non attivo"
    except Exception as e:
        return False, f"Errore: {e}"

def get_robot_status(robot_ip: str) -> dict:
    """Ottiene lo stato completo del robot."""
    status = {}
    
    commands = {
        "robotmode": "robotmode",
        "safetymode": "safetymode",
        "programState": "programState",
        "remote_control": "is in remote control"
    }
    
    for key, cmd in commands.items():
        success, response = send_dashboard_command(robot_ip, cmd)
        if success:
            status[key] = response
        else:
            status[key] = f"ERROR: {response}"
    
    return status

def main():
    print("=" * 70)
    print("VERIFICA VNC E POWER ON SUL ROBOT UR5e")
    print("=" * 70)
    print(f"Robot IP: {ROBOT_IP}\n")
    
    # ========================================================================
    # 1. VERIFICA CONNESSIONE ROBOT
    # ========================================================================
    print("[1/4] Verifica connessione robot...")
    if check_port(ROBOT_IP, DASHBOARD_PORT):
        print(f"  [OK] Dashboard Server raggiungibile su porta {DASHBOARD_PORT}")
    else:
        print(f"  [ERR] Dashboard Server NON raggiungibile su porta {DASHBOARD_PORT}")
        print(f"     Verifica che il robot sia acceso e connesso alla rete")
        return 1
    
    # ========================================================================
    # 2. VERIFICA VNC
    # ========================================================================
    print("\n[2/4] Verifica VNC sul robot...")
    if check_port(ROBOT_IP, VNC_PORT):
        print(f"  [OK] VNC Server attivo su porta {VNC_PORT}")
        print(f"     Puoi connetterti con: vncviewer {ROBOT_IP}:{VNC_PORT}")
    else:
        print(f"  [WARN] VNC Server NON attivo su porta {VNC_PORT}")
        print(f"     Il robot potrebbe non avere VNC installato o configurato")
    
    # Verifica anche porta 5901 (display :1)
    if check_port(ROBOT_IP, 5901):
        print(f"  [OK] VNC Server attivo anche su porta 5901 (display :1)")
    
    # ========================================================================
    # 3. VERIFICA STATO ATTUALE ROBOT
    # ========================================================================
    print("\n[3/4] Stato attuale robot...")
    status = get_robot_status(ROBOT_IP)
    
    print(f"  Robot Mode: {status.get('robotmode', 'N/A')}")
    print(f"  Safety Mode: {status.get('safetymode', 'N/A')}")
    print(f"  Program State: {status.get('programState', 'N/A')}")
    print(f"  Remote Control: {status.get('remote_control', 'N/A')}")
    
    robotmode = status.get('robotmode', '')
    
    # ========================================================================
    # 4. TEST POWER ON
    # ========================================================================
    print("\n[4/4] Test comando Power On...")
    
    if "POWER_OFF" in robotmode:
        print("  [INFO] Robot e in POWER_OFF - testero il comando power on")
        
        print("  [TEST] Invio comando 'power on'...")
        success, response = send_dashboard_command(ROBOT_IP, "power on")
        
        if success:
            print(f"  [RESP] Risposta: {response}")
            
            if "Powering on" in response or "powering on" in response.lower():
                print("  [OK] Comando accettato! Attendo 5 secondi...")
                time.sleep(5)
                
                # Verifica nuovo stato
                print("  [CHECK] Verifica nuovo stato...")
                new_status = get_robot_status(ROBOT_IP)
                new_robotmode = new_status.get('robotmode', '')
                
                print(f"  [STATUS] Nuovo Robot Mode: {new_robotmode}")
                
                if "POWER_ON" in new_robotmode or "IDLE" in new_robotmode:
                    print("  [SUCCESS] POWER ON FUNZIONA! Robot acceso con successo!")
                else:
                    print(f"  [WARN] Robot non ancora in POWER_ON (attuale: {new_robotmode})")
                    print("     Potrebbe richiedere più tempo o ci potrebbe essere un problema")
            elif "already" in response.lower() or "already powered" in response.lower():
                print("  [INFO] Robot gia acceso")
            else:
                print(f"  [WARN] Risposta inaspettata: {response}")
        else:
            print(f"  [ERR] Errore invio comando: {response}")
            return 1
    elif "POWER_ON" in robotmode or "IDLE" in robotmode or "RUNNING" in robotmode:
        print("  [INFO] Robot e gia acceso - testero comunque il comando")
        
        print("  [TEST] Invio comando 'power on' (dovrebbe rispondere 'already powered')...")
        success, response = send_dashboard_command(ROBOT_IP, "power on")
        
        if success:
            print(f"  [RESP] Risposta: {response}")
            if "already" in response.lower() or "already powered" in response.lower():
                print("  [OK] Comando funziona correttamente (robot gia acceso)")
            else:
                print(f"  [WARN] Risposta inaspettata: {response}")
        else:
            print(f"  [ERR] Errore invio comando: {response}")
            return 1
    else:
        print(f"  [WARN] Stato robot sconosciuto: {robotmode}")
        print("  [TEST] Provo comunque il comando power on...")
        
        success, response = send_dashboard_command(ROBOT_IP, "power on")
        if success:
            print(f"  [RESP] Risposta: {response}")
        else:
            print(f"  [ERR] Errore: {response}")
    
    # ========================================================================
    # RIEPILOGO FINALE
    # ========================================================================
    print("\n" + "=" * 70)
    print("RIEPILOGO")
    print("=" * 70)
    
    final_status = get_robot_status(ROBOT_IP)
    print(f"[OK] Dashboard Server: Raggiungibile")
    vnc_active = check_port(ROBOT_IP, VNC_PORT)
    print(f"[{'OK' if vnc_active else 'ERR'}] VNC Server: {'Attivo' if vnc_active else 'Non attivo'}")
    print(f"[STATUS] Robot Mode: {final_status.get('robotmode', 'N/A')}")
    print(f"[STATUS] Safety Mode: {final_status.get('safetymode', 'N/A')}")
    print(f"[STATUS] Remote Control: {final_status.get('remote_control', 'N/A')}")
    
    print("\n[INFO] Per connetterti via VNC:")
    print(f"   vncviewer {ROBOT_IP}:{VNC_PORT}")
    print(f"   oppure usa TightVNC Viewer su Windows")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
