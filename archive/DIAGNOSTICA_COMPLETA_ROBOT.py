#!/usr/bin/env python3
"""
Sistema di diagnostica completo per capire perché il robot non si muove
Analizza tutti i problemi e fornisce soluzioni dettagliate
"""

import requests
import socket
import time
import json
import subprocess
import sys
from datetime import datetime

WEB_URL = "http://192.168.10.191:8081"
ROBOT_IP = "192.168.10.194"

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'
    BOLD = '\033[1m'

def print_header(text):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.END}\n")

def print_ok(text):
    print(f"{Colors.GREEN}✅ {text}{Colors.END}")

def print_error(text):
    print(f"{Colors.RED}❌ {text}{Colors.END}")

def print_warning(text):
    print(f"{Colors.YELLOW}⚠️  {text}{Colors.END}")

def print_info(text):
    print(f"{Colors.BLUE}ℹ️  {text}{Colors.END}")

problems = []
solutions = []

# ============================================================================
# 1. VERIFICA STATO ROBOT
# ============================================================================
print_header("1. VERIFICA STATO ROBOT")

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ROBOT_IP, 29999))
    sock.recv(1024)
    
    sock.sendall(b"robotmode\n")
    robot_mode = sock.recv(1024).decode().strip()
    print(f"   Robot Mode: {robot_mode}")
    
    sock.sendall(b"programState\n")
    program_state = sock.recv(1024).decode().strip()
    print(f"   Program State: {program_state}")
    
    sock.sendall(b"isInRemoteControl\n")
    remote_control = sock.recv(1024).decode().strip()
    print(f"   Remote Control: {remote_control}")
    
    sock.close()
    
    if "RUNNING" in robot_mode:
        print_ok("Robot in RUNNING")
    else:
        print_error(f"Robot NON in RUNNING: {robot_mode}")
        problems.append(f"Robot non in RUNNING: {robot_mode}")
        solutions.append("Accendi il robot o mettilo in RUNNING")
    
    if "PLAYING" in program_state:
        print_ok("Programma in PLAYING")
    else:
        print_error(f"Programma NON in PLAYING: {program_state}")
        problems.append(f"Programma non in PLAYING: {program_state}")
        solutions.append("Avvia il programma sul teach pendant e mettilo in PLAYING")
    
    if "true" in remote_control.lower():
        print_ok("Remote Control attivo")
    else:
        print_warning(f"Remote Control: {remote_control}")
        problems.append(f"Remote Control potrebbe non essere attivo: {remote_control}")
        solutions.append("Verifica che Remote Control sia attivo sul teach pendant")
        
except Exception as e:
    print_error(f"Errore connessione Dashboard: {e}")
    problems.append(f"Impossibile connettersi al robot: {e}")
    solutions.append("Verifica connessione di rete e che il robot sia acceso")

# ============================================================================
# 2. VERIFICA WEB INTERFACE
# ============================================================================
print_header("2. VERIFICA WEB INTERFACE")

try:
    r = requests.get(f"{WEB_URL}/api/status", timeout=5)
    if r.status_code == 200:
        print_ok("Web interface raggiungibile")
        data = r.json().get('data', {})
        bridge = data.get('ros2_bridge', {})
        
        ros_init = bridge.get('ros_initialized', False)
        loop_running = bridge.get('publish_loop_running', False)
        rate = bridge.get('publish_rate_hz', 0)
        last_command = bridge.get('last_command_age_s', None)
        last_publish = bridge.get('last_publish_age_s', None)
        
        print(f"   ROS2 inizializzato: {ros_init}")
        print(f"   Publish loop attivo: {loop_running}")
        print(f"   Frequenza: {rate}Hz")
        print(f"   Ultimo comando: {last_command}s fa" if last_command is not None else "   Ultimo comando: mai")
        print(f"   Ultimo publish: {last_publish}s fa" if last_publish is not None else "   Ultimo publish: mai")
        
        if not loop_running:
            print_error("Publish loop NON attivo!")
            problems.append("ROS2 publish loop non attivo")
            solutions.append("Riavvia la web interface")
        
        if last_command and last_command > 5:
            print_warning(f"Ultimo comando troppo vecchio: {last_command:.1f}s fa")
            problems.append(f"Ultimo comando troppo vecchio: {last_command:.1f}s fa")
            solutions.append("Muovi il joystick nella web interface per inviare comandi")
        
    else:
        print_error(f"Web interface errore: {r.status_code}")
        problems.append(f"Web interface non raggiungibile: {r.status_code}")
        solutions.append("Riavvia la web interface")
        
except Exception as e:
    print_error(f"Errore web interface: {e}")
    problems.append(f"Impossibile connettersi alla web interface: {e}")
    solutions.append("Verifica che la web interface sia avviata")

# ============================================================================
# 3. VERIFICA ROS2 TOPICS E PUBBLICAZIONE
# ============================================================================
print_header("3. VERIFICA ROS2 TOPICS")

try:
    # Verifica se ROS2 topics esistono
    cmd = "source /opt/ros/humble/setup.bash && ros2 topic list 2>&1 | grep -E 'forward_velocity|scaled_joint_trajectory' || echo 'NO_TOPICS'"
    result = subprocess.run(['ssh', '-o', 'StrictHostKeyChecking=no', 'lab@192.168.10.191', cmd], 
                          capture_output=True, text=True, timeout=10)
    
    topics = result.stdout.strip()
    if "forward_velocity" in topics or "scaled_joint_trajectory" in topics:
        print_ok("Topic ROS2 trovati")
        print(f"   {topics}")
    else:
        print_warning("Topic ROS2 non trovati o driver UR ROS2 non in esecuzione")
        problems.append("Topic ROS2 non trovati - driver UR ROS2 probabilmente non in esecuzione")
        solutions.append("Avvia driver UR ROS2: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194")
        
except Exception as e:
    print_warning(f"Impossibile verificare topic ROS2: {e}")
    print_info("Questo è normale se ROS2 non è configurato")

# ============================================================================
# 4. TEST INVIO COMANDO DIRETTO SOCKET
# ============================================================================
print_header("4. TEST INVIO COMANDO DIRETTO SOCKET")

try:
    print_info("Invio comando diretto via socket (porta 30002)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((ROBOT_IP, 30002))
    
    # Piccolo movimento joint 1
    script = "speedj([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\n"
    sock.sendall(script.encode('utf-8'))
    print_ok("Comando inviato via socket")
    print_info("💡 Il robot dovrebbe muoversi leggermente (joint 1)")
    
    time.sleep(0.5)
    
    # Stop
    script = "stopj(1.0)\n"
    sock.sendall(script.encode('utf-8'))
    sock.close()
    
    print_ok("Test completato")
    print_info("Se il robot si è mosso → Socket funziona, problema è nella web interface")
    print_info("Se il robot NON si è mosso → Problema è nel robot/configurazione")
    
except Exception as e:
    print_error(f"Errore invio comando socket: {e}")
    problems.append(f"Impossibile inviare comando via socket: {e}")
    solutions.append("Verifica che il programma sia in PLAYING e che la porta 30002 sia aperta")

# ============================================================================
# 5. TEST INVIO COMANDO VIA WEB INTERFACE
# ============================================================================
print_header("5. TEST INVIO COMANDO VIA WEB INTERFACE")

try:
    print_info("Invio comando via web interface API...")
    payload = {
        "speeds": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
        "cartesian": False
    }
    r = requests.post(f"{WEB_URL}/api/servo_loop_update", json=payload, timeout=5)
    
    print(f"   Status code: {r.status_code}")
    
    if r.status_code == 200:
        data = r.json()
        print_ok("Comando accettato dalla web interface")
        print(f"   Messaggio: {data.get('message', 'OK')}")
        print_info("💡 Verifica se il robot si muove")
    elif r.status_code == 400:
        data = r.json()
        msg = data.get('message', '')
        print_error(f"Errore 400: {msg}")
        problems.append(f"Web interface rifiuta comando: {msg}")
        solutions.append(msg)
    else:
        print_error(f"Errore {r.status_code}")
        print(f"   Risposta: {r.text[:200]}")
        problems.append(f"Web interface errore {r.status_code}")
        solutions.append("Controlla log web interface: tail -f /tmp/web_interface.log")
        
except Exception as e:
    print_error(f"Errore test web interface: {e}")
    problems.append(f"Impossibile testare web interface: {e}")
    solutions.append("Verifica che la web interface sia avviata")

# ============================================================================
# 6. VERIFICA CONNESSIONI PORTE
# ============================================================================
print_header("6. VERIFICA PORTE ROBOT")

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
            print_ok(f"Porta {port} ({name}): APERTA")
        else:
            if port == 50002:
                print_warning(f"Porta {port} ({name}): CHIUSA (normale se ROS2 non configurato)")
            else:
                print_error(f"Porta {port} ({name}): CHIUSA")
                problems.append(f"Porta {port} ({name}) chiusa")
                solutions.append(f"Verifica configurazione robot per porta {port}")
    except Exception as e:
        if port != 50002:
            print_error(f"Porta {port} ({name}): Errore - {e}")

# ============================================================================
# 7. ANALISI FINALE E SOLUZIONI
# ============================================================================
print_header("ANALISI FINALE E SOLUZIONI")

if not problems:
    print_ok("Nessun problema trovato!")
    print_info("Se il robot ancora non si muove:")
    print_info("1. Verifica che il programma sul teach pendant sia effettivamente in PLAYING")
    print_info("2. Prova a muovere il joystick nella web interface")
    print_info("3. Controlla log web interface: tail -f /tmp/web_interface.log")
else:
    print_error(f"Trovati {len(problems)} problema/i:")
    for i, problem in enumerate(problems, 1):
        print(f"\n{i}. {problem}")
        if i <= len(solutions):
            print(f"   💡 SOLUZIONE: {solutions[i-1]}")

print_header("RACCOMANDAZIONI")

print_info("Per controllo via SOCKET (più semplice):")
print("   1. Assicurati che programma sia in PLAYING")
print("   2. La web interface usa già socket diretto")
print("   3. Muovi joystick - dovrebbe funzionare")

print_info("\nPer controllo via ROS2 (più complesso):")
print("   1. Installa External Control URCap sul robot")
print("   2. Configura programma con External Control (IP: 192.168.10.191, Porta: 50002)")
print("   3. Avvia driver UR ROS2:")
print("      ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194")
print("   4. Riavvia web interface con ROS2 configurato")

print_header("LOG COMPLETO SALVATO")

log_data = {
    "timestamp": datetime.now().isoformat(),
    "robot_ip": ROBOT_IP,
    "web_url": WEB_URL,
    "problems": problems,
    "solutions": solutions,
    "robot_mode": robot_mode if 'robot_mode' in locals() else "unknown",
    "program_state": program_state if 'program_state' in locals() else "unknown",
    "remote_control": remote_control if 'remote_control' in locals() else "unknown"
}

with open("diagnostica_robot_log.json", "w") as f:
    json.dump(log_data, f, indent=2)

print_ok("Log salvato in: diagnostica_robot_log.json")
print("\n")










