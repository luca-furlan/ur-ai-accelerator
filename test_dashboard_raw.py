#!/usr/bin/env python3
"""Test RAW del Dashboard Server per vedere esattamente cosa restituisce."""

import socket
import time

ROBOT_IP = "192.168.10.194"
DASHBOARD_PORT = 29999

def test_raw():
    print("=" * 70)
    print("TEST RAW DASHBOARD SERVER - Lettura esatta delle risposte")
    print("=" * 70)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10.0)  # Timeout più lungo
    sock.connect((ROBOT_IP, DASHBOARD_PORT))
    
    # Leggi welcome
    print("\n1. WELCOME MESSAGE:")
    welcome = sock.recv(1024)
    print(f"   Raw bytes: {welcome}")
    print(f"   Decoded: '{welcome.decode('utf-8', errors='ignore')}'")
    
    # Test robotmode - con lettura multipla
    print("\n2. TEST ROBOTMODE (con lettura multipla):")
    sock.sendall(b"robotmode\n")
    
    # Prova a leggere più volte per vedere tutte le risposte
    responses = []
    for i in range(5):
        try:
            sock.settimeout(0.5)
            data = sock.recv(1024)
            if data:
                responses.append(data)
                print(f"   Lettura {i+1}: {data} -> '{data.decode('utf-8', errors='ignore').strip()}'")
            else:
                break
        except socket.timeout:
            break
    
    sock.settimeout(10.0)
    
    # Test safetymode
    print("\n3. TEST SAFETYMODE:")
    sock.sendall(b"safetymode\n")
    time.sleep(0.3)
    try:
        data = sock.recv(1024)
        print(f"   Raw: {data} -> '{data.decode('utf-8', errors='ignore').strip()}'")
    except:
        print("   Nessuna risposta")
    
    # Test programState
    print("\n4. TEST PROGRAMSTATE:")
    sock.sendall(b"programState\n")
    time.sleep(0.3)
    try:
        data = sock.recv(1024)
        print(f"   Raw: {data} -> '{data.decode('utf-8', errors='ignore').strip()}'")
    except:
        print("   Nessuna risposta")
    
    # Test con nuova connessione per ogni comando
    print("\n5. TEST CON NUOVA CONNESSIONE PER OGNI COMANDO:")
    commands = ["robotmode", "safetymode", "programState", "is in remote control"]
    for cmd in commands:
        try:
            new_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_sock.settimeout(5.0)
            new_sock.connect((ROBOT_IP, DASHBOARD_PORT))
            welcome = new_sock.recv(1024)  # Leggi welcome
            new_sock.sendall((cmd + "\n").encode('utf-8'))
            time.sleep(0.3)
            response = new_sock.recv(1024)
            print(f"   {cmd}: {response} -> '{response.decode('utf-8', errors='ignore').strip()}'")
            new_sock.close()
        except Exception as e:
            print(f"   {cmd}: ERRORE - {e}")
    
    sock.close()
    print("\n" + "=" * 70)

if __name__ == "__main__":
    test_raw()

