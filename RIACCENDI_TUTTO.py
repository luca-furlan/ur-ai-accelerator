#!/usr/bin/env python3
"""
Script da eseguire SULL'AI ACCELERATOR per riavviare tutto
Esegui: ssh lab@192.168.10.191 "cd ~/MekoAiAccelerator && python3 RIACCENDI_TUTTO.py"
"""

import os
import sys
import subprocess
import time

print("=" * 80)
print("RIAVVIO COMPLETO WEB INTERFACE")
print("=" * 80)
print()

# 1. Ferma tutto
print("1. Fermo processi esistenti...")
try:
    subprocess.run(["pkill", "-9", "-f", "web_interface"], 
                   capture_output=True, timeout=5)
    time.sleep(2)
    print("   ✅ Processi fermati")
except:
    print("   ⚠️  Nessun processo trovato")

# 2. Vai nella directory
os.chdir(os.path.expanduser("~/MekoAiAccelerator"))
print(f"   Directory: {os.getcwd()}")

# 3. Configura variabili
os.environ['UR_ROBOT_IP'] = '192.168.10.194'
os.environ['WEB_PORT'] = '8081'
os.environ['WEB_HOST'] = '0.0.0.0'

print("\n2. Configurazione:")
print(f"   UR_ROBOT_IP: {os.environ['UR_ROBOT_IP']}")
print(f"   WEB_PORT: {os.environ['WEB_PORT']}")
print(f"   WEB_HOST: {os.environ['WEB_HOST']}")

# 4. Avvia web interface
print("\n3. Avvio web interface...")
try:
    process = subprocess.Popen(
        [sys.executable, "-m", "remote_ur_control.web_interface"],
        stdout=open("/tmp/web_interface.log", "w"),
        stderr=subprocess.STDOUT,
        env=os.environ,
        cwd=os.getcwd()
    )
    
    # Salva PID
    with open("/tmp/web_interface.pid", "w") as f:
        f.write(str(process.pid))
    
    print(f"   ✅ Web interface avviata (PID: {process.pid})")
    print(f"   URL: http://192.168.10.191:8081")
    
    # Aspetta
    time.sleep(5)
    
    # Verifica
    if process.poll() is None:
        print("   ✅ Web interface in esecuzione")
        print("\n" + "=" * 80)
        print("✅ TUTTO RIAVVIATO!")
        print("=" * 80)
        print("\nApri il browser su: http://192.168.10.191:8081")
        print("\nDovresti vedere:")
        print("  - ROS2 Monitor")
        print("  - Robot Status (con joints e TCP pose)")
        print("  - Joystick e controlli")
    else:
        print("   ❌ Web interface terminata")
        print("\n   Log:")
        try:
            with open("/tmp/web_interface.log", "r") as f:
                print(f.read()[-500:])
        except:
            pass
        sys.exit(1)
        
except Exception as e:
    print(f"   ❌ Errore: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print()




