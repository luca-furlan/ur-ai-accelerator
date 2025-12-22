#!/usr/bin/env python3
"""
Verifica dettagliata dello stato della macchina AI Accelerator
"""

import socket
import subprocess
import sys

def check_port(ip, port, timeout=2):
    """Verifica se una porta è aperta"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except:
        return False

def ping_host(ip):
    """Esegue ping"""
    try:
        result = subprocess.run(
            ['ping', '-n', '2', '-w', '2000', ip],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except:
        return False

def main():
    ip = "192.168.10.191"
    
    print("=" * 70)
    print("VERIFICA DETTAGLIATA MACCHINA AI ACCELERATOR")
    print("=" * 70)
    print(f"IP: {ip}")
    print()
    
    # 1. Ping
    print("1. PING...")
    ping_ok = ping_host(ip)
    if ping_ok:
        print("   ✅ Macchina risponde al ping")
    else:
        print("   ❌ Macchina NON risponde al ping")
    print()
    
    # 2. Porte comuni
    print("2. VERIFICA PORTE...")
    ports = {
        22: "SSH",
        80: "HTTP",
        443: "HTTPS",
        5901: "VNC",
        8080: "Web Interface",
        8081: "Web Interface (alt)",
    }
    
    for port, name in ports.items():
        status = check_port(ip, port)
        if status:
            print(f"   ✅ Porta {port} ({name}): APERTA")
        else:
            print(f"   ❌ Porta {port} ({name}): CHIUSA")
    print()
    
    # 3. Diagnosi
    print("3. DIAGNOSI...")
    print()
    
    if not ping_ok:
        print("   ⚠️ La macchina non risponde al ping.")
        print("      Possibili cause:")
        print("      - Macchina ancora in avvio (boot in corso)")
        print("      - Macchina in stato di errore/crash")
        print("      - Problema di rete/firewall")
        print("      - Macchina in standby/sospesa")
        print()
        print("   💡 SOLUZIONI:")
        print("      1. Aspetta 2-3 minuti e riprova")
        print("      2. Verifica fisicamente la macchina (LED, ventole)")
        print("      3. Se possibile, collega monitor/tastiera direttamente")
        print()
    
    if ping_ok and not check_port(ip, 22):
        print("   ⚠️ La macchina risponde al ping ma SSH non è attivo.")
        print("      Possibili cause:")
        print("      - Servizio SSH non avviato")
        print("      - SSH configurato su porta diversa")
        print("      - Firewall blocca porta 22")
        print()
        print("   💡 SOLUZIONI:")
        print("      Se hai accesso fisico:")
        print("      - sudo systemctl start ssh")
        print("      - sudo systemctl enable ssh")
        print("      - sudo ufw allow 22/tcp")
        print()
    
    if ping_ok and check_port(ip, 22):
        print("   ✅ Macchina operativa e SSH disponibile!")
        print(f"      Connettiti con: ssh lab@{ip}")
        print()
    
    print("=" * 70)

if __name__ == "__main__":
    main()











