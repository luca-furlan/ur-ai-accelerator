#!/usr/bin/env python3
"""
Monitor continuo dello stato robot - mostra problemi in tempo reale
"""

import requests
import socket
import time
import json
from datetime import datetime

WEB_URL = "http://192.168.10.191:8081"
ROBOT_IP = "192.168.10.194"

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    END = '\033[0m'
    BOLD = '\033[1m'
    CLEAR = '\033[2J\033[H'

def print_status(title, status, color=Colors.GREEN):
    print(f"{color}{Colors.BOLD}{title}:{Colors.END} {status}")

def check_robot_status():
    """Verifica stato robot via Dashboard"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        sock.connect((ROBOT_IP, 29999))
        sock.recv(1024)
        
        sock.sendall(b"robotmode\n")
        robot_mode = sock.recv(1024).decode().strip()
        
        sock.sendall(b"programState\n")
        program_state = sock.recv(1024).decode().strip()
        
        sock.close()
        return robot_mode, program_state
    except Exception as e:
        return None, f"Errore: {e}"

def check_web_interface():
    """Verifica web interface"""
    try:
        r = requests.get(f"{WEB_URL}/api/status", timeout=3)
        if r.status_code == 200:
            return r.json().get('data', {})
        return None
    except:
        return None

def check_robot_status_api():
    """Verifica stato robot via API"""
    try:
        r = requests.get(f"{WEB_URL}/api/robot_status", timeout=5)
        if r.status_code == 200:
            return r.json().get('data', {})
        return None
    except:
        return None

def test_command():
    """Test invio comando"""
    try:
        payload = {"speeds": [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], "cartesian": False}
        r = requests.post(f"{WEB_URL}/api/servo_loop_update", json=payload, timeout=3)
        if r.status_code == 200:
            data = r.json()
            return data.get('message', 'OK')
        return f"Errore {r.status_code}"
    except Exception as e:
        return f"Errore: {e}"

def main():
    print(f"{Colors.CLEAR}{Colors.BOLD}{Colors.CYAN}{'='*80}")
    print("MONITOR CONTINUO ROBOT - Aggiornamento ogni 2 secondi")
    print(f"{'='*80}{Colors.END}\n")
    
    last_command_time = None
    
    while True:
        try:
            # Stato robot diretto
            robot_mode, program_state = check_robot_status()
            
            # Stato web interface
            web_data = check_web_interface()
            robot_api = check_robot_status_api()
            
            # Test comando
            cmd_result = test_command()
            
            # Timestamp
            now = datetime.now().strftime("%H:%M:%S")
            
            print(f"{Colors.CLEAR}{Colors.BOLD}{Colors.CYAN}{'='*80}")
            print(f"MONITOR ROBOT - {now} (CTRL+C per uscire)")
            print(f"{'='*80}{Colors.END}\n")
            
            # Robot Status
            print(f"{Colors.BOLD}ROBOT STATUS:{Colors.END}")
            if robot_mode:
                if "RUNNING" in robot_mode:
                    print_status("  Robot Mode", robot_mode, Colors.GREEN)
                else:
                    print_status("  Robot Mode", robot_mode, Colors.RED)
            else:
                print_status("  Robot Mode", "NON RAGGIUNGIBILE", Colors.RED)
            
            if program_state:
                if "PLAYING" in program_state:
                    print_status("  Program State", program_state, Colors.GREEN)
                else:
                    print_status("  Program State", program_state, Colors.RED)
                    print(f"{Colors.RED}  ⚠️  PROBLEMA: Programma NON in PLAYING!{Colors.END}")
            else:
                print_status("  Program State", "NON RAGGIUNGIBILE", Colors.RED)
            
            # Web Interface Status
            print(f"\n{Colors.BOLD}WEB INTERFACE:{Colors.END}")
            if web_data:
                bridge = web_data.get('ros2_bridge', {})
                loop_running = bridge.get('publish_loop_running', False)
                last_cmd = bridge.get('last_command_age_s', None)
                
                if loop_running:
                    print_status("  Publish Loop", "ATTIVO", Colors.GREEN)
                else:
                    print_status("  Publish Loop", "FERMO", Colors.YELLOW)
                
                if last_cmd is not None:
                    if last_cmd < 5:
                        print_status(f"  Ultimo Comando", f"{last_cmd:.1f}s fa", Colors.GREEN)
                    else:
                        print_status(f"  Ultimo Comando", f"{last_cmd:.1f}s fa (TROPPO VECCHIO!)", Colors.RED)
                        print(f"{Colors.RED}  ⚠️  PROBLEMA: Nessun comando recente!{Colors.END}")
            else:
                print_status("  Web Interface", "NON RAGGIUNGIBILE", Colors.RED)
            
            # Test Comando
            print(f"\n{Colors.BOLD}TEST COMANDO:{Colors.END}")
            if "Socket control" in cmd_result:
                print_status("  Risultato", cmd_result, Colors.GREEN)
                print(f"{Colors.GREEN}  ✅ Socket funziona correttamente!{Colors.END}")
            elif "ROS2" in cmd_result:
                print_status("  Risultato", cmd_result, Colors.YELLOW)
                print(f"{Colors.YELLOW}  ⚠️  Sta usando ROS2 invece di socket!{Colors.END}")
            elif "Errore" in cmd_result:
                print_status("  Risultato", cmd_result, Colors.RED)
                print(f"{Colors.RED}  ❌ Errore invio comando!{Colors.END}")
            else:
                print_status("  Risultato", cmd_result, Colors.YELLOW)
            
            # Robot API Status
            if robot_api:
                dashboard = robot_api.get('dashboard', {})
                rtde = robot_api.get('rtde', {})
                
                print(f"\n{Colors.BOLD}ROBOT API:{Colors.END}")
                if rtde.get('joints'):
                    joints = rtde['joints']
                    print(f"  Joint Positions: {[round(j, 3) for j in joints]}")
                if rtde.get('error'):
                    print_status("  RTDE Error", rtde['error'], Colors.RED)
            
            # Diagnosi
            print(f"\n{Colors.BOLD}DIAGNOSI:{Colors.END}")
            issues = []
            
            if program_state and "PLAYING" not in program_state:
                issues.append("❌ Programma NON in PLAYING")
            
            if web_data:
                bridge = web_data.get('ros2_bridge', {})
                last_cmd = bridge.get('last_command_age_s', None)
                if last_cmd and last_cmd > 5:
                    issues.append("❌ Nessun comando recente (muovi joystick!)")
            
            if "ROS2" in cmd_result:
                issues.append("⚠️  Sta usando ROS2 invece di socket")
            
            if not issues:
                print(f"{Colors.GREEN}  ✅ Tutto OK! Robot dovrebbe muoversi.{Colors.END}")
            else:
                for issue in issues:
                    print(f"  {issue}")
            
            print(f"\n{Colors.CYAN}{'─'*80}{Colors.END}")
            print(f"{Colors.YELLOW}Premi CTRL+C per uscire{Colors.END}\n")
            
            time.sleep(2)
            
        except KeyboardInterrupt:
            print(f"\n{Colors.CYAN}Monitor fermato.{Colors.END}\n")
            break
        except Exception as e:
            print(f"{Colors.RED}Errore: {e}{Colors.END}\n")
            time.sleep(2)

if __name__ == "__main__":
    main()




