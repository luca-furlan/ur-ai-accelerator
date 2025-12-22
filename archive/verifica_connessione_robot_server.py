#!/usr/bin/env python3
"""
Verifica connessione dal robot al server sulla porta 50002.
Simula quello che fa il robot quando cerca di connettersi.
"""

import socket
import sys
import time

ROBOT_IP = "192.168.10.194"  # IP del robot (per verificare che possa raggiungere il server)
SERVER_IP = "192.168.10.191"  # IP del server dove deve connettersi
EXTERNAL_CONTROL_PORT = 50002  # Porta External Control

def test_connection_from_robot_perspective():
    """Testa la connessione come se fossimo il robot."""
    print("=" * 70)
    print("VERIFICA CONNESSIONE ROBOT -> SERVER (PORTA 50002)")
    print("=" * 70)
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Server IP: {SERVER_IP}")
    print(f"Porta: {EXTERNAL_CONTROL_PORT}")
    print("")
    
    print("[TEST] Tentativo connessione dal robot al server...")
    print(f"  Simulo: Robot ({ROBOT_IP}) -> Server ({SERVER_IP}:{EXTERNAL_CONTROL_PORT})")
    print("")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        
        print(f"  [1] Connessione a {SERVER_IP}:{EXTERNAL_CONTROL_PORT}...")
        sock.connect((SERVER_IP, EXTERNAL_CONTROL_PORT))
        print(f"  [OK] Connessione stabilita!")
        
        print(f"  [2] Attendo risposta dal server...")
        time.sleep(0.5)
        
        # Prova a leggere qualcosa (il driver ROS2 potrebbe inviare dati)
        sock.settimeout(2.0)
        try:
            data = sock.recv(1024)
            if data:
                print(f"  [OK] Dati ricevuti dal server: {data[:50]}...")
            else:
                print(f"  [INFO] Nessun dato ricevuto (normale per External Control)")
        except socket.timeout:
            print(f"  [INFO] Nessun dato ricevuto (normale - il server aspetta comandi)")
        
        sock.close()
        
        print("")
        print("[SUCCESS] Connessione funzionante!")
        print("  Il robot DOVREBBE riuscire a connettersi se:")
        print("    1. Remote Control e attivo sul robot")
        print("    2. Il programma ha IP corretto (192.168.10.191)")
        print("    3. Il programma ha porta corretta (50002)")
        
        return True
        
    except ConnectionRefusedError:
        print(f"  [ERR] Connessione rifiutata!")
        print("")
        print("[PROBLEMA] Il server rifiuta la connessione")
        print("  Possibili cause:")
        print("    1. Driver ROS2 non attivo sul server")
        print("    2. Porta 50002 non in ascolto")
        print("    3. Firewall blocca la connessione")
        return False
        
    except socket.timeout:
        print(f"  [ERR] Timeout connessione!")
        print("")
        print("[PROBLEMA] Il server non risponde")
        print("  Possibili cause:")
        print("    1. Server non raggiungibile")
        print("    2. Porta 50002 non aperta")
        print("    3. Firewall blocca la connessione")
        return False
        
    except Exception as e:
        print(f"  [ERR] Errore: {e}")
        return False

def main():
    success = test_connection_from_robot_perspective()
    
    print("")
    print("=" * 70)
    print("RIEPILOGO")
    print("=" * 70)
    
    if success:
        print("[OK] Connessione TCP funzionante")
        print("[INFO] Se il robot ancora non si connette:")
        print("  1. Verifica Remote Control attivo sul robot")
        print("  2. Verifica configurazione programma (IP e porta)")
        print("  3. Riavvia il programma sul robot")
    else:
        print("[ERR] Connessione NON funzionante")
        print("[AZIONE] Avvia il driver ROS2 sul server!")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
