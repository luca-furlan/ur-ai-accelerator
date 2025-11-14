"""
Forza l'avvio del programma sul robot.
"""

import os
import socket
import sys
import time

def force_play(robot_ip: str):
    """Forza l'avvio del programma."""
    print(f"Connessione Dashboard Server ({robot_ip}:29999)...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((robot_ip, 29999))
    
    # Welcome
    welcome = sock.recv(1024)
    print(f"Connesso: {welcome.decode('utf-8').strip()}")
    
    # Comandi sequenziali
    commands = [
        ("robotmode", "Verifica modalita robot..."),
        ("programState", "Verifica stato programma..."),
        ("stop", "Stop programma (se in esecuzione)..."),
        ("play", "Avvio programma..."),
    ]
    
    for cmd, desc in commands:
        print(f"\n{desc}")
        sock.sendall((cmd + "\n").encode('utf-8'))
        time.sleep(0.3)
        response = sock.recv(1024).decode('utf-8').strip()
        print(f"  Risposta: {response}")
    
    sock.close()
    
    print("\n[OK] Comandi inviati")
    print("\nVerifica sul teach pendant che il programma sia PLAYING")

if __name__ == "__main__":
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    force_play(robot_ip)

