"""
Accende il robot e avvia il programma.
"""

import os
import socket
import sys
import time

def power_on_and_play(robot_ip: str):
    """Accende il robot e avvia il programma."""
    print("=" * 60)
    print("ACCENSIONE E AVVIO ROBOT")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((robot_ip, 29999))
    
    # Welcome
    welcome = sock.recv(1024)
    print(f"Connesso: {welcome.decode('utf-8').strip()}\n")
    
    # Sequenza di comandi
    steps = [
        ("robotmode", "1. Verifica modalita..."),
        ("power on", "2. Accensione robot..."),
    ]
    
    for cmd, desc in steps:
        print(desc)
        sock.sendall((cmd + "\n").encode('utf-8'))
        time.sleep(0.5)
        response = sock.recv(1024).decode('utf-8').strip()
        print(f"   Risposta: {response}\n")
    
    # Attesa accensione
    print("3. Attesa 5 secondi per l'accensione...")
    time.sleep(5)
    
    # Verifica nuovo stato
    print("\n4. Verifica nuovo stato...")
    sock.sendall(b"robotmode\n")
    time.sleep(0.3)
    response = sock.recv(1024).decode('utf-8').strip()
    print(f"   Robot Mode: {response}")
    
    # Brake release se necessario
    if "POWER_ON" in response or "IDLE" in response:
        print("\n5. Rilascio freni...")
        sock.sendall(b"brake release\n")
        time.sleep(0.3)
        response = sock.recv(1024).decode('utf-8').strip()
        print(f"   Risposta: {response}")
        time.sleep(2)
    
    # Play programma
    print("\n6. Avvio programma...")
    sock.sendall(b"play\n")
    time.sleep(0.5)
    response = sock.recv(1024).decode('utf-8').strip()
    print(f"   Risposta: {response}")
    
    if "Starting program" in response or "Playing" in response:
        print("\n7. Attesa 3 secondi...")
        time.sleep(3)
        
        # Verifica stato finale
        print("\n8. Verifica stato finale...")
        sock.sendall(b"robotmode\n")
        time.sleep(0.3)
        robotmode = sock.recv(1024).decode('utf-8').strip()
        
        sock.sendall(b"programState\n")
        time.sleep(0.3)
        program_state = sock.recv(1024).decode('utf-8').strip()
        
        print(f"   Robot Mode: {robotmode}")
        print(f"   Program State: {program_state}")
        
        if "RUNNING" in robotmode and "PLAYING" in program_state:
            print("\n[OK] Robot pronto!")
        else:
            print("\n[ATTENZIONE] Verifica manualmente sul teach pendant")
    
    sock.close()
    
    print("\n" + "=" * 60)
    print("PROCESSO COMPLETATO")
    print("=" * 60)

if __name__ == "__main__":
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("\n[ATTENZIONE] Questo script tentera di accendere il robot.")
    print("Assicurati che:")
    print("  - Il teach pendant sia acceso")
    print("  - Non ci siano operatori nell'area")
    print("  - L'e-stop sia raggiungibile")
    print("\nPremere INVIO per continuare o CTRL+C per annullare...")
    
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("\nOperazione annullata")
        sys.exit(1)
    
    power_on_and_play(robot_ip)

