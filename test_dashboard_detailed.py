#!/usr/bin/env python3
"""Test dettagliato del Dashboard Server per vedere esattamente cosa restituisce."""

import socket
import time

ROBOT_IP = "192.168.10.194"
ROBOT_DASHBOARD_PORT = 29999

def test_dashboard():
    print("=" * 70)
    print("TEST DETTAGLIATO DASHBOARD SERVER")
    print("=" * 70)
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Porta: {ROBOT_DASHBOARD_PORT}\n")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((ROBOT_IP, ROBOT_DASHBOARD_PORT))
        
        # Leggi messaggio di benvenuto
        print("1. Messaggio di benvenuto:")
        welcome = sock.recv(1024).decode('utf-8', errors='ignore')
        print(f"   '{welcome}'")
        print(f"   (lunghezza: {len(welcome)} bytes)")
        print(f"   (bytes raw: {welcome.encode('utf-8')})\n")
        
        # Test comandi uno per uno
        commands = [
            "robotmode",
            "safetymode",
            "programState",
            "is in remote control",
            "get loaded program",
            "popup",
            "version"
        ]
        
        print("2. Risposte ai comandi:")
        for cmd in commands:
            try:
                print(f"\n   Comando: '{cmd}'")
                sock.sendall((cmd + "\n").encode('utf-8'))
                time.sleep(0.2)
                
                # Prova a leggere più volte per vedere se ci sono più messaggi
                response = b""
                sock.settimeout(0.5)
                try:
                    while True:
                        chunk = sock.recv(1024)
                        if not chunk:
                            break
                        response += chunk
                except socket.timeout:
                    pass
                
                sock.settimeout(5.0)  # Reset timeout
                
                response_str = response.decode('utf-8', errors='ignore').strip()
                print(f"   Risposta: '{response_str}'")
                print(f"   (lunghezza: {len(response_str)} chars)")
                print(f"   (bytes raw: {response})")
                
                # Analisi della risposta
                if response_str:
                    parts = response_str.split()
                    print(f"   (parti separate: {parts})")
                    
            except Exception as e:
                print(f"   ERRORE: {e}")
        
        sock.close()
        
    except Exception as e:
        print(f"ERRORE CONNESSIONE: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_dashboard()

