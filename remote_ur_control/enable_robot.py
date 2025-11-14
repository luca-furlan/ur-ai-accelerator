"""
Script per abilitare il robot e prepararlo per il controllo remoto.
Controlla lo stato e avvia il robot se necessario.
"""

import os
import socket
import sys
import time

def send_dashboard_command(robot_ip: str, command: str, port: int = 29999):
    """Invia un comando al Dashboard Server e ritorna la risposta."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((robot_ip, port))
        
        # Leggi welcome message
        sock.recv(1024)
        
        # Invia comando
        sock.sendall((command + "\n").encode('utf-8'))
        time.sleep(0.2)
        
        # Leggi risposta
        response = sock.recv(1024).decode('utf-8').strip()
        sock.close()
        return response
    except Exception as e:
        return f"ERROR: {e}"

def get_robot_status(robot_ip: str):
    """Ottiene lo stato completo del robot."""
    status = {}
    commands = {
        "robotmode": "robotmode",
        "safetymode": "safetymode",
        "programState": "programState",
        "remote_control": "is in remote control"
    }
    
    for key, cmd in commands.items():
        response = send_dashboard_command(robot_ip, cmd)
        status[key] = response
        time.sleep(0.1)
    
    return status

def enable_robot(robot_ip: str):
    """Abilita il robot per il controllo remoto."""
    print("=" * 60)
    print("ABILITAZIONE ROBOT UR")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    # Step 1: Verifica stato attuale
    print("[STEP 1] Verifica stato attuale...")
    status = get_robot_status(robot_ip)
    
    robotmode = status.get("robotmode", "")
    safetymode = status.get("safetymode", "")
    program_state = status.get("programState", "")
    remote_control = status.get("remote_control", "")
    
    print(f"  Robot Mode: {robotmode}")
    print(f"  Safety Mode: {safetymode}")
    print(f"  Program State: {program_state}")
    print(f"  Remote Control: {remote_control}\n")
    
    # Step 2: Power On se necessario
    if "POWER_OFF" in robotmode:
        print("[STEP 2] Robot spento - accensione...")
        response = send_dashboard_command(robot_ip, "power on")
        print(f"  Risposta: {response}")
        print("  Attesa 5 secondi per l'accensione...")
        time.sleep(5)
        
        # Verifica nuovo stato
        status = get_robot_status(robot_ip)
        robotmode = status.get("robotmode", "")
        print(f"  Nuovo Robot Mode: {robotmode}\n")
    
    # Step 3: Brake Release se necessario
    if "POWER_ON" in robotmode or "IDLE" in robotmode:
        print("[STEP 3] Rilascio freni...")
        response = send_dashboard_command(robot_ip, "brake release")
        print(f"  Risposta: {response}")
        print("  Attesa 2 secondi...")
        time.sleep(2)
    
    # Step 4: Verifica se c'è un programma caricato
    print("[STEP 4] Verifica programma...")
    loaded_program = send_dashboard_command(robot_ip, "get loaded program")
    print(f"  Programma caricato: {loaded_program}")
    
    if "<unnamed>" in loaded_program or "No program loaded" in loaded_program:
        print("\n[ATTENZIONE] Nessun programma valido caricato!")
        print("  Soluzione:")
        print("  1. Sul teach pendant, crea o carica un programma")
        print("  2. Il programma deve essere in modalita REMOTE")
        print("  3. Poi esegui di nuovo questo script")
        return False
    
    # Step 5: Play programma
    print("\n[STEP 5] Avvio programma...")
    response = send_dashboard_command(robot_ip, "play")
    print(f"  Risposta: {response}")
    
    if "Starting program" in response or "Playing" in response:
        print("  Attesa 3 secondi per l'avvio...")
        time.sleep(3)
        
        # Verifica stato finale
        status = get_robot_status(robot_ip)
        robotmode = status.get("robotmode", "")
        program_state = status.get("programState", "")
        
        print(f"\n  Robot Mode: {robotmode}")
        print(f"  Program State: {program_state}")
        
        if "RUNNING" in robotmode and "PLAYING" in program_state:
            print("\n[OK] Robot pronto per il controllo remoto!")
            return True
        else:
            print("\n[ATTENZIONE] Robot non completamente pronto")
            print(f"  Verifica manualmente sul teach pendant")
            return False
    else:
        print(f"\n[ERR] Impossibile avviare programma: {response}")
        return False

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("\n[ATTENZIONE] Assicurarsi che:")
    print("  - Il teach pendant sia acceso")
    print("  - Non ci siano operatori nell'area di lavoro")
    print("  - L'e-stop sia facilmente raggiungibile")
    print("\nPremere INVIO per continuare o CTRL+C per annullare...")
    
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("\nOperazione annullata")
        return 1
    
    success = enable_robot(robot_ip)
    
    if success:
        print("\n" + "=" * 60)
        print("ROBOT PRONTO!")
        print("=" * 60)
        print("\nOra puoi eseguire i test di movimento:")
        print("  python -m remote_ur_control.test_all_commands")
        return 0
    else:
        print("\n" + "=" * 60)
        print("ATTENZIONE")
        print("=" * 60)
        print("\nIl robot potrebbe non essere completamente pronto.")
        print("Verifica manualmente sul teach pendant e riprova.")
        return 1

if __name__ == "__main__":
    sys.exit(main())

