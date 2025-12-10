#!/usr/bin/env python3
"""
External Control Proxy Server
Gestisce la connessione dal robot e attende che il driver ROS2 sia pronto.
Risolve il problema del "Connection refused" quando il driver va in crash.
"""

import socket
import threading
import time
import sys
import subprocess

# Configurazione
PC_IP = "192.168.10.191"
PORT = 50002
DRIVER_PORT = 50002  # Porta dove il driver ROS2 si metterà in ascolto
MAX_RETRIES = 60  # Numero massimo di tentativi (60 secondi)
RETRY_DELAY = 1  # Secondi tra un tentativo e l'altro

def check_driver_ready():
    """Verifica se il driver ROS2 è in ascolto sulla porta"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex((PC_IP, DRIVER_PORT))
        sock.close()
        return result == 0
    except:
        return False

def wait_for_driver():
    """Attende che il driver ROS2 sia pronto"""
    print(f"[PROXY] Attendo che il driver ROS2 sia in ascolto sulla porta {DRIVER_PORT}...")
    for i in range(MAX_RETRIES):
        if check_driver_ready():
            print(f"[PROXY] ✅ Driver ROS2 pronto!")
            return True
        if i % 10 == 0:
            print(f"[PROXY] ⏳ Attesa... ({i}/{MAX_RETRIES})")
        time.sleep(RETRY_DELAY)
    print(f"[PROXY] ❌ Timeout: driver ROS2 non disponibile dopo {MAX_RETRIES} secondi")
    return False

def handle_client(client_socket, client_address):
    """Gestisce una connessione client"""
    print(f"[PROXY] Nuova connessione da {client_address}")
    
    # Attendi che il driver sia pronto
    if not wait_for_driver():
        print(f"[PROXY] ❌ Impossibile connettersi al driver ROS2")
        client_socket.close()
        return
    
    # Connetti al driver ROS2
    try:
        driver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        driver_socket.connect((PC_IP, DRIVER_PORT))
        print(f"[PROXY] ✅ Connesso al driver ROS2")
        
        # Proxy bidirezionale
        def forward_data(source, destination, direction):
            try:
                while True:
                    data = source.recv(4096)
                    if not data:
                        break
                    destination.sendall(data)
                    if len(data) > 0:
                        print(f"[PROXY] {direction}: {len(data)} bytes")
            except Exception as e:
                print(f"[PROXY] Errore forwarding {direction}: {e}")
            finally:
                try:
                    source.close()
                    destination.close()
                except:
                    pass
        
        # Avvia forwarding in entrambe le direzioni
        t1 = threading.Thread(target=forward_data, args=(client_socket, driver_socket, "Robot→Driver"))
        t2 = threading.Thread(target=forward_data, args=(driver_socket, client_socket, "Driver→Robot"))
        
        t1.daemon = True
        t2.daemon = True
        
        t1.start()
        t2.start()
        
        # Attendi che uno dei thread finisca
        t1.join()
        t2.join()
        
        print(f"[PROXY] Connessione con {client_address} chiusa")
        
    except Exception as e:
        print(f"[PROXY] ❌ Errore connessione al driver: {e}")
        client_socket.close()

def main():
    """Server principale"""
    print("=" * 70)
    print("EXTERNAL CONTROL PROXY SERVER")
    print("=" * 70)
    print(f"IP: {PC_IP}")
    print(f"Porta: {PORT}")
    print(f"Driver ROS2 porta: {DRIVER_PORT}")
    print("=" * 70)
    print()
    print("Questo server:")
    print("1. Si mette in ascolto sulla porta 50002")
    print("2. Accetta connessioni dal robot")
    print("3. Attende che il driver ROS2 sia pronto")
    print("4. Fa da proxy tra robot e driver ROS2")
    print()
    print("Premi CTRL+C per fermare")
    print()
    
    # Crea socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind(('', PORT))
        server_socket.listen(5)
        print(f"[PROXY] ✅ Server in ascolto sulla porta {PORT}")
        print()
        
        while True:
            client_socket, client_address = server_socket.accept()
            # Gestisci ogni client in un thread separato
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, client_address)
            )
            client_thread.daemon = True
            client_thread.start()
            
    except KeyboardInterrupt:
        print("\n[PROXY] Arresto server...")
    except Exception as e:
        print(f"[PROXY] ❌ Errore: {e}")
    finally:
        server_socket.close()
        print("[PROXY] Server chiuso")

if __name__ == '__main__':
    main()

