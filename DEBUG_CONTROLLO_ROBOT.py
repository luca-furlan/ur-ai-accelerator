#!/usr/bin/env python3
"""
Debug completo perché il robot non si muove
"""

import requests
import socket
import time

WEB_URL = "http://192.168.10.191:8081"
ROBOT_IP = "192.168.10.194"

print("=" * 80)
print("DEBUG: PERCHÉ IL ROBOT NON SI MUOVE?")
print("=" * 80)
print()

# 1. Verifica stato robot
print("1. STATO ROBOT...")
try:
    r = requests.get(f"{WEB_URL}/api/robot_status", timeout=10)
    if r.status_code == 200:
        data = r.json().get('data', {})
        dashboard = data.get('dashboard', {})
        
        robot_mode = dashboard.get('robotmode', 'unknown')
        program_state = dashboard.get('programState', 'unknown')
        remote_control = dashboard.get('remote_control', 'unknown')
        
        print(f"   Robot Mode: {robot_mode}")
        print(f"   Program State: {program_state}")
        print(f"   Remote Control: {remote_control}")
        print()
        
        if "PLAYING" not in program_state:
            print("   ❌ PROBLEMA: Programma NON in PLAYING!")
            print("   💡 SOLUZIONE: Avvia il programma sul teach pendant")
            print()
        
        if "RUNNING" not in robot_mode:
            print("   ❌ PROBLEMA: Robot NON in RUNNING!")
            print()
    else:
        print(f"   ❌ Errore API: {r.status_code}")
except Exception as e:
    print(f"   ❌ Errore: {e}")

# 2. Verifica ROS2 bridge
print("2. ROS2 BRIDGE...")
try:
    r = requests.get(f"{WEB_URL}/api/status", timeout=5)
    if r.status_code == 200:
        data = r.json().get('data', {})
        bridge = data.get('ros2_bridge', {})
        
        ros_init = bridge.get('ros_initialized', False)
        loop_running = bridge.get('publish_loop_running', False)
        rate = bridge.get('publish_rate_hz', 0)
        last_publish = bridge.get('last_publish_age_s', None)
        last_error = bridge.get('last_error', None)
        
        print(f"   ROS2 inizializzato: {ros_init}")
        print(f"   Publish loop attivo: {loop_running}")
        print(f"   Frequenza: {rate}Hz")
        print(f"   Ultimo publish: {last_publish}s fa" if last_publish is not None else "   Ultimo publish: mai")
        
        if last_error:
            print(f"   ❌ ERRORE: {last_error}")
        
        if not loop_running:
            print("   ❌ PROBLEMA: Publish loop NON attivo!")
            print("   💡 SOLUZIONE: Riavvia la web interface")
            print()
        
        if last_publish and last_publish > 1.0:
            print(f"   ⚠️  PROBLEMA: Ultimo publish {last_publish:.1f}s fa - troppo vecchio!")
            print()
    else:
        print(f"   ❌ Errore API: {r.status_code}")
except Exception as e:
    print(f"   ❌ Errore: {e}")

# 3. Test invio comando con dettagli
print("3. TEST INVIO COMANDO...")
try:
    payload = {
        "speeds": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],  # Piccolo movimento joint 1
        "cartesian": False
    }
    print(f"   Invio comando: {payload['speeds']}")
    r = requests.post(f"{WEB_URL}/api/servo_loop_update", json=payload, timeout=5)
    print(f"   Status code: {r.status_code}")
    
    if r.status_code == 200:
        data = r.json()
        print(f"   ✅ Comando accettato")
        print(f"   Messaggio: {data.get('message', 'OK')}")
    elif r.status_code == 400:
        data = r.json()
        msg = data.get('message', '')
        print(f"   ❌ ERRORE 400: {msg}")
    else:
        print(f"   ❌ ERRORE {r.status_code}")
        print(f"   Risposta: {r.text[:300]}")
except Exception as e:
    print(f"   ❌ Errore: {e}")
    import traceback
    traceback.print_exc()

# 4. Verifica connessione robot
print()
print("4. CONNESSIONE ROBOT...")
ports_to_check = [
    (30001, "Primary Interface"),
    (30002, "URScript Port"),
    (30004, "RTDE"),
    (29999, "Dashboard Server"),
    (50002, "ROS2 Control (se configurato)")
]

for port, name in ports_to_check:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((ROBOT_IP, port))
        sock.close()
        if result == 0:
            print(f"   ✅ Porta {port} ({name}): APERTA")
        else:
            print(f"   ❌ Porta {port} ({name}): CHIUSA")
    except Exception as e:
        print(f"   ❌ Porta {port} ({name}): Errore - {e}")

# 5. Verifica ROS2 topics (se ROS2 disponibile)
print()
print("5. VERIFICA ROS2 TOPICS...")
print("   (Esegui su AI Accelerator: ros2 topic list)")
print("   Dovresti vedere:")
print("   - /joint_group_vel_controller/commands")
print("   - /scaled_joint_trajectory_controller/commands")
print("   - /tf")
print()

# 6. Diagnosi finale
print("=" * 80)
print("DIAGNOSI FINALE")
print("=" * 80)
print()
print("PROBLEMI COMUNI:")
print()
print("1. Programma NON in PLAYING sul teach pendant")
print("   → SOLUZIONE: Avvia il programma sul teach pendant")
print()
print("2. ROS2 Control non configurato sul robot")
print("   → SOLUZIONE: Installa External Control URCap sul robot")
print("   → SOLUZIONE: Configura IP: 192.168.10.191, Porta: 50002")
print()
print("3. Publish loop non attivo")
print("   → SOLUZIONE: Riavvia la web interface")
print()
print("4. Robot non raggiungibile")
print("   → SOLUZIONE: Verifica connessione di rete")
print()
print("=" * 80)




