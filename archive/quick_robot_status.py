#!/usr/bin/env python3
"""Verifica rapida dello stato del robot e tentativo di accensione se necessario."""

import socket
import time

ROBOT_IP = "192.168.10.194"
DASHBOARD_PORT = 29999

def send_command(sock, cmd):
    """Invia un comando e ritorna la risposta."""
    sock.sendall((cmd + "\n").encode('utf-8'))
    time.sleep(0.2)
    response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    return response

def main():
    print("=" * 70)
    print("VERIFICA RAPIDA STATO ROBOT")
    print("=" * 70)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((ROBOT_IP, DASHBOARD_PORT))
    
    # Welcome
    welcome = sock.recv(1024)
    print(f"Connesso: {welcome.decode('utf-8').strip()}\n")
    
    # Stato attuale
    print("STATO ATTUALE:")
    robotmode = send_command(sock, "robotmode")
    safetymode = send_command(sock, "safetymode")
    program_state = send_command(sock, "programState")
    remote_control = send_command(sock, "is in remote control")
    
    print(f"  Robot Mode: {robotmode}")
    print(f"  Safety Mode: {safetymode}")
    print(f"  Program State: {program_state}")
    print(f"  Remote Control: {remote_control}\n")
    
    # Se è in POWER_OFF, chiedi se accenderlo
    if "POWER_OFF" in robotmode:
        print("⚠️  Robot risulta in POWER_OFF")
        print("\nVuoi provare ad accenderlo? (s/n): ", end="")
        try:
            response = input().strip().lower()
            if response == 's':
                print("\nTentativo di accensione...")
                power_response = send_command(sock, "power on")
                print(f"  Risposta: {power_response}")
                
                if "Powering on" in power_response or "OK" in power_response:
                    print("  Attesa 5 secondi...")
                    time.sleep(5)
                    
                    # Verifica nuovo stato
                    print("\nNuovo stato:")
                    robotmode = send_command(sock, "robotmode")
                    print(f"  Robot Mode: {robotmode}")
                    
                    if "POWER_ON" in robotmode or "IDLE" in robotmode:
                        print("\n✅ Robot acceso!")
                        
                        # Prova brake release
                        print("\nRilascio freni...")
                        brake_response = send_command(sock, "brake release")
                        print(f"  Risposta: {brake_response}")
                        time.sleep(2)
                        
                        # Prova play
                        print("\nAvvio programma...")
                        play_response = send_command(sock, "play")
                        print(f"  Risposta: {play_response}")
                        time.sleep(2)
                        
                        # Stato finale
                        print("\nSTATO FINALE:")
                        robotmode = send_command(sock, "robotmode")
                        program_state = send_command(sock, "programState")
                        print(f"  Robot Mode: {robotmode}")
                        print(f"  Program State: {program_state}")
            else:
                print("\nOperazione annullata")
        except (KeyboardInterrupt, EOFError):
            print("\nOperazione annullata")
    else:
        print("✅ Robot non in POWER_OFF")
    
    sock.close()
    print("\n" + "=" * 70)

if __name__ == "__main__":
    main()

