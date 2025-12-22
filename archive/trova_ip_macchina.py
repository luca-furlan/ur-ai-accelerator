#!/usr/bin/env python3
"""
Script per trovare la macchina AI Accelerator sulla rete locale
scansionando gli IP nella subnet 192.168.10.0/24
"""

import subprocess
import socket
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

def ping_host(ip):
    """Esegue ping su un host"""
    try:
        # Windows
        result = subprocess.run(
            ['ping', '-n', '1', '-w', '1000', ip],
            capture_output=True,
            timeout=2
        )
        return result.returncode == 0
    except:
        return False

def check_ssh_port(ip, port=22, timeout=1):
    """Verifica se la porta SSH è aperta"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except:
        return False

def check_hostname(ip):
    """Prova a risolvere il reverse DNS"""
    try:
        hostname = socket.gethostbyaddr(ip)[0]
        return hostname
    except:
        return None

def scan_network():
    """Scansiona la rete 192.168.10.0/24"""
    print("=" * 70)
    print("SCANSIONE RETE LOCALE - Ricerca AI Accelerator")
    print("=" * 70)
    print()
    print("Scansione IP 192.168.10.1 - 192.168.10.254...")
    print("(Questo può richiedere 1-2 minuti)")
    print()
    
    base_ip = "192.168.10"
    found_hosts = []
    
    # Scansiona tutti gli IP
    with ThreadPoolExecutor(max_workers=50) as executor:
        futures = []
        for i in range(1, 255):
            ip = f"{base_ip}.{i}"
            future = executor.submit(check_host, ip)
            futures.append((future, ip))
        
        completed = 0
        for future, ip in futures:
            try:
                result = future.result(timeout=3)
                if result:
                    found_hosts.append(result)
                completed += 1
                if completed % 20 == 0:
                    print(f"  Scansione: {completed}/254...", end='\r')
            except:
                pass
    
    print()
    print()
    
    if found_hosts:
        print("=" * 70)
        print("HOST TROVATI CON SSH ATTIVO:")
        print("=" * 70)
        print()
        for host in found_hosts:
            print(f"  ✅ {host['ip']}")
            if host['hostname']:
                print(f"     Hostname: {host['hostname']}")
            print(f"     SSH: Porta 22 aperta")
            print()
        
        # Cerca specificamente "ubuntu" o "jetson"
        ai_accelerator = None
        for host in found_hosts:
            if host['hostname'] and ('ubuntu' in host['hostname'].lower() or 'jetson' in host['hostname'].lower()):
                ai_accelerator = host
                break
        
        if ai_accelerator:
            print("=" * 70)
            print("🎯 AI ACCELERATOR PROBABILE:")
            print("=" * 70)
            print(f"   IP: {ai_accelerator['ip']}")
            print(f"   Hostname: {ai_accelerator['hostname']}")
            print()
            print(f"   Prova: ssh lab@{ai_accelerator['ip']}")
            print()
        else:
            print("💡 Nessun hostname 'ubuntu' o 'jetson' trovato.")
            print("   Controlla manualmente gli IP sopra.")
    else:
        print("=" * 70)
        print("❌ NESSUN HOST CON SSH TROVATO")
        print("=" * 70)
        print()
        print("Possibili cause:")
        print("  - Macchina spenta o non connessa")
        print("  - Macchina su subnet diversa")
        print("  - Firewall blocca le connessioni")
        print("  - Servizio SSH non attivo")
        print()
    
    print("=" * 70)

def check_host(ip):
    """Verifica un singolo host"""
    # Prima ping veloce
    if not ping_host(ip):
        return None
    
    # Poi verifica SSH
    if not check_ssh_port(ip):
        return None
    
    # Se SSH è aperto, prova hostname
    hostname = check_hostname(ip)
    
    return {
        'ip': ip,
        'hostname': hostname
    }

if __name__ == "__main__":
    try:
        scan_network()
    except KeyboardInterrupt:
        print("\n\nScansione interrotta dall'utente.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nErrore durante la scansione: {e}")
        sys.exit(1)











