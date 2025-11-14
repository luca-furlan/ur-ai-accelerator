"""
Script per verificare lo stato del robot e diagnosticare perché non si muove.
"""

import os
import socket
import sys
import time

def check_dashboard(robot_ip: str, port: int = 29999):
    """Verifica lo stato del robot tramite Dashboard Server."""
    print(f"\n[CHECK] Connessione Dashboard Server ({robot_ip}:{port})...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((robot_ip, port))
        
        # Leggi messaggio di benvenuto
        welcome = sock.recv(1024).decode('utf-8')
        print(f"  Welcome: {welcome.strip()}")
        
        # Comandi Dashboard
        commands = [
            "robotmode",
            "safetymode", 
            "programState",
            "get loaded program",
            "is in remote control"
        ]
        
        results = {}
        for cmd in commands:
            sock.sendall((cmd + "\n").encode('utf-8'))
            time.sleep(0.1)
            response = sock.recv(1024).decode('utf-8').strip()
            results[cmd] = response
            print(f"  {cmd}: {response}")
        
        sock.close()
        return results
    except Exception as e:
        print(f"  [ERR] Errore Dashboard: {e}")
        return None

def test_script_execution(robot_ip: str, port: int = 30002):
    """Test se gli script vengono eseguiti correttamente."""
    print(f"\n[CHECK] Test esecuzione script URScript ({robot_ip}:{port})...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((robot_ip, port))
        
        # Script che fa solo un beep e log
        test_script = (
            "def test_exec():\n"
            '  textmsg("TEST_SCRIPT_EXECUTION")\n'
            "end\n"
            "test_exec()\n"
        )
        
        print("  Invio script di test...")
        sock.sendall(test_script.encode('utf-8'))
        time.sleep(0.5)
        sock.close()
        print("  [OK] Script inviato")
        return True
    except Exception as e:
        print(f"  [ERR] Errore: {e}")
        return False

def test_movej_direct(robot_ip: str, port: int = 30002):
    """Test movej con script più esplicito."""
    print(f"\n[CHECK] Test MoveJ diretto...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((robot_ip, port))
        
        # Script movej più esplicito con get_actual_joint_positions
        script = (
            "def test_move():\n"
            "  current = get_actual_joint_positions()\n"
            "  target = [current[0] + 0.1, current[1], current[2], current[3], current[4], current[5]]\n"
            "  movej(target, a=0.5, v=0.1)\n"
            "end\n"
            "test_move()\n"
        )
        
        print("  Invio comando movej (joint 0 + 0.1 rad)...")
        print(f"  Script:\n{script}")
        sock.sendall(script.encode('utf-8'))
        time.sleep(0.5)
        sock.close()
        print("  [OK] Comando inviato")
        print("  [INFO] Controlla il robot - dovrebbe muoversi!")
        return True
    except Exception as e:
        print(f"  [ERR] Errore: {e}")
        return False

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("DIAGNOSTICA STATO ROBOT UR")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    # Check 1: Dashboard Server
    dashboard_status = check_dashboard(robot_ip)
    
    # Check 2: Test script execution
    test_script_execution(robot_ip)
    
    # Check 3: Test movej diretto
    print("\n" + "=" * 60)
    print("TEST MOVIMENTO DIRETTO")
    print("=" * 60)
    print("\n[ATTENZIONE] Il robot dovrebbe muoversi ora!")
    print("Premere INVIO per continuare con il test movimento...")
    try:
        input()
    except KeyboardInterrupt:
        print("\nTest annullato")
        return 1
    
    test_movej_direct(robot_ip)
    
    print("\n" + "=" * 60)
    print("RIEPILOGO")
    print("=" * 60)
    
    if dashboard_status:
        robotmode = dashboard_status.get("robotmode", "unknown")
        safetymode = dashboard_status.get("safetymode", "unknown")
        program_state = dashboard_status.get("programState", "unknown")
        remote_control = dashboard_status.get("is in remote control", "unknown")
        
        print(f"\nRobot Mode: {robotmode}")
        print(f"Safety Mode: {safetymode}")
        print(f"Program State: {program_state}")
        print(f"Remote Control: {remote_control}")
        
        if robotmode != "RUNNING":
            print("\n[PROBLEMA] Robot non in modalita RUNNING!")
            print("  Soluzione: Avviare un programma sul teach pendant")
        
        if safetymode != "NORMAL":
            print(f"\n[PROBLEMA] Safety Mode non NORMAL: {safetymode}")
        
        if program_state != "PLAYING":
            print(f"\n[PROBLEMA] Programma non in PLAYING: {program_state}")
            print("  Soluzione: Premere PLAY sul teach pendant")
        
        if remote_control != "true":
            print(f"\n[PROBLEMA] Robot non in remote control: {remote_control}")
            print("  Soluzione: Attivare remote control sul teach pendant")
    
    print("\n[INFO] Se il robot non si muove ancora:")
    print("  1. Verificare che ci sia un programma attivo sul teach pendant")
    print("  2. Il programma deve essere in modalita REMOTE")
    print("  3. Premere PLAY sul teach pendant")
    print("  4. Verificare che non ci siano errori sul teach pendant")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

