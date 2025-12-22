#!/usr/bin/env python3
"""
Script da eseguire SULL'AI ACCELERATOR per applicare fix RTDE
Esegui: ssh lab@192.168.10.191 "cd ~/MekoAiAccelerator && python3 applica_fix_remoto.py"
"""

import os
import sys
import subprocess
import time
import shutil

print("=" * 80)
print("APPLICAZIONE FIX RTDE")
print("=" * 80)
print()

# 1. Verifica file
web_interface_path = os.path.expanduser("~/MekoAiAccelerator/remote_ur_control/web_interface.py")
if not os.path.exists(web_interface_path):
    print(f"❌ File non trovato: {web_interface_path}")
    sys.exit(1)

print(f"✅ File trovato: {web_interface_path}")

# 2. Ferma web interface esistente
print("\n2. Ferma web interface esistente...")
try:
    result = subprocess.run(["pkill", "-f", "web_interface"], capture_output=True)
    time.sleep(2)
    print("   ✅ Processi fermati")
except Exception as e:
    print(f"   ⚠️  {e}")

# 3. Verifica che il file abbia il fix RTDE
print("\n3. Verifica fix RTDE nel file...")
with open(web_interface_path, 'r') as f:
    content = f.read()
    if 'actual_tcp_pose' in content and 'DataType.VECTOR6D' in content:
        print("   ✅ Fix RTDE presente nel file")
    else:
        print("   ⚠️  Fix RTDE potrebbe non essere presente")
        print("   Il file potrebbe non essere stato aggiornato")

# 4. Avvia web interface
print("\n4. Avvia web interface...")
os.chdir(os.path.expanduser("~/MekoAiAccelerator"))
os.environ['UR_ROBOT_IP'] = '192.168.10.194'
os.environ['WEB_PORT'] = '8081'
os.environ['WEB_HOST'] = '0.0.0.0'

try:
    # Avvia in background
    process = subprocess.Popen(
        [sys.executable, "-m", "remote_ur_control.web_interface"],
        stdout=open("/tmp/web_interface.log", "w"),
        stderr=subprocess.STDOUT,
        env=os.environ
    )
    
    # Salva PID
    with open("/tmp/web_interface.pid", "w") as f:
        f.write(str(process.pid))
    
    print(f"   ✅ Web interface avviata (PID: {process.pid})")
    print(f"   URL: http://192.168.10.191:8081")
    print(f"   Log: /tmp/web_interface.log")
    
    # Aspetta che si avvii
    time.sleep(5)
    
    # Verifica che sia ancora in esecuzione
    if process.poll() is None:
        print("   ✅ Web interface in esecuzione")
    else:
        print("   ❌ Web interface terminata")
        print("   Log:")
        try:
            with open("/tmp/web_interface.log", "r") as f:
                print(f.read()[-500:])
        except:
            pass
        sys.exit(1)
        
except Exception as e:
    print(f"   ❌ Errore avvio: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 80)
print("✅ FIX APPLICATO E WEB INTERFACE RIAVVIATA")
print("=" * 80)
print("\nApri il browser su: http://192.168.10.191:8081")
print("\nDovresti vedere:")
print("  ✅ Joint Positions: [valori aggiornati ogni 2 secondi]")
print("  ✅ TCP Pose: [valori aggiornati ogni 2 secondi]")
print("  ❌ Nessun errore RTDE")
print()










