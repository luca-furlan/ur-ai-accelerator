#!/usr/bin/env python3
"""Verifica lo stato reale del robot tramite diversi metodi."""

import socket
import subprocess
import sys
import time

ROBOT_IP = "192.168.10.194"

def check_via_ssh():
    """Verifica lo stato del robot tramite SSH."""
    print("\n1. VERIFICA TRAMITE SSH:")
    try:
        # Prova a connettere via SSH e vedere processi attivi
        ssh_cmd = f'ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 root@{ROBOT_IP} "ps aux | grep -i ur | head -5"'
        result = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"   Processi UR attivi:\n{result.stdout}")
            return True
        else:
            print(f"   Errore SSH: {result.stderr}")
            return False
    except Exception as e:
        print(f"   Errore: {e}")
        return False

def check_via_urscript():
    """Verifica lo stato tramite porta URScript (30002)."""
    print("\n2. VERIFICA TRAMITE URSCRIPT PORT (30002):")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((ROBOT_IP, 30002))
        
        # Invia uno script che legge lo stato reale
        script = (
            "def get_status():\n"
            "  textmsg(\"=== ROBOT STATUS ===\")\n"
            "  textmsg(\"Robot mode: \" + robot_mode())\n"
            "  textmsg(\"Safety mode: \" + safety_mode())\n"
            "  textmsg(\"Program running: \" + to_str(running()))\n"
            "  textmsg(\"Remote control: \" + to_str(is_remote_control()))\n"
            "  joints = get_actual_joint_positions()\n"
            "  textmsg(\"Joint positions: \" + to_str(joints))\n"
            "end\n"
            "get_status()\n"
        )
        
        sock.sendall(script.encode('utf-8'))
        time.sleep(1)
        
        # Prova a leggere eventuali risposte
        try:
            sock.settimeout(1.0)
            response = sock.recv(4096)
            if response:
                print(f"   Risposta: {response.decode('utf-8', errors='ignore')}")
        except socket.timeout:
            print("   Nessuna risposta testuale (normale per URScript)")
        
        sock.close()
        print("   ✅ Connessione URScript OK - script inviato")
        print("   (Controlla i log sul teach pendant per vedere lo stato)")
        return True
        
    except Exception as e:
        print(f"   ❌ Errore: {e}")
        return False

def check_dashboard_alternative():
    """Verifica Dashboard Server con comandi alternativi."""
    print("\n3. VERIFICA DASHBOARD CON COMANDI ALTERNATIVI:")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((ROBOT_IP, 29999))
        
        welcome = sock.recv(1024)
        
        # Prova comandi alternativi
        alt_commands = [
            "robotmode",
            "get robot mode",
            "state",
            "status",
            "get loaded program",
            "running",
            "is program running"
        ]
        
        for cmd in alt_commands:
            try:
                sock.sendall((cmd + "\n").encode('utf-8'))
                time.sleep(0.2)
                response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
                if response and "could not understand" not in response.lower():
                    print(f"   {cmd}: {response}")
            except:
                pass
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"   Errore: {e}")
        return False

def main():
    print("=" * 70)
    print("VERIFICA STATO REALE ROBOT - Metodi Alternativi")
    print("=" * 70)
    print(f"Robot IP: {ROBOT_IP}\n")
    
    # Metodo 1: SSH
    check_via_ssh()
    
    # Metodo 2: URScript
    check_via_urscript()
    
    # Metodo 3: Dashboard alternativi
    check_dashboard_alternative()
    
    print("\n" + "=" * 70)
    print("NOTA: Se il robot è in esecuzione programma ma il Dashboard")
    print("      mostra POWER_OFF, potrebbe essere un problema di")
    print("      sincronizzazione del Dashboard Server.")
    print("=" * 70)

if __name__ == "__main__":
    main()

