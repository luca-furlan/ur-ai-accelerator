#!/usr/bin/env python3
"""
Script semplice per avviare la web interface
Esegui questo script SULL'AI ACCELERATOR (via SSH)
"""

import os
import sys
import subprocess
import time

# Configurazione
os.environ['UR_ROBOT_IP'] = '192.168.10.194'
os.environ['WEB_PORT'] = '8081'
os.environ['WEB_HOST'] = '0.0.0.0'

print("=" * 80)
print("AVVIO WEB INTERFACE")
print("=" * 80)
print(f"Robot IP: {os.environ['UR_ROBOT_IP']}")
print(f"Web Port: {os.environ['WEB_PORT']}")
print(f"Web Host: {os.environ['WEB_HOST']}")
print("=" * 80)
print()

# Cambia directory
os.chdir(os.path.expanduser('~/MekoAiAccelerator'))

# Avvia web interface
print("Avvio web interface...")
print(f"URL: http://{os.environ['WEB_HOST']}:{os.environ['WEB_PORT']}")
print()
print("Premi CTRL+C per fermare")
print()

try:
    # Importa e avvia
    from remote_ur_control.web_interface import main
    main()
except KeyboardInterrupt:
    print("\n\nWeb interface fermata")
except Exception as e:
    print(f"\n❌ Errore: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)




