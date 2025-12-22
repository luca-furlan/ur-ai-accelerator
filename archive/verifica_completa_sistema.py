#!/usr/bin/env python3
"""
Script completo per verificare tutto il sistema:
- Connessione AI Accelerator
- Web Interface
- Stato Robot
- API Endpoints
"""

import subprocess
import sys
import socket
import requests
import time
from pathlib import Path

AI_ACCELERATOR_IP = "192.168.10.191"
ROBOT_IP = "192.168.10.194"
WEB_PORT = 8081
WEB_URL = f"http://{AI_ACCELERATOR_IP}:{WEB_PORT}"

def test_ssh_connection():
    """Test connessione SSH all'AI Accelerator"""
    print("=" * 80)
    print("1. VERIFICA CONNESSIONE SSH AI ACCELERATOR")
    print("=" * 80)
    try:
        result = subprocess.run(
            ['ssh', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=no',
             f'lab@{AI_ACCELERATOR_IP}', 'echo "OK"'],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0 and "OK" in result.stdout:
            print(f"✅ SSH connesso a {AI_ACCELERATOR_IP}")
            return True
        else:
            print(f"❌ SSH fallito: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"❌ Timeout connessione SSH")
        return False
    except FileNotFoundError:
        print("⚠️  SSH non disponibile (normale su Windows)")
        print("   Puoi usare PuTTY o WSL per connetterti")
        return None
    except Exception as e:
        print(f"❌ Errore SSH: {e}")
        return False

def test_ping():
    """Test ping AI Accelerator e Robot"""
    print("\n" + "=" * 80)
    print("2. VERIFICA PING")
    print("=" * 80)
    
    results = {}
    
    # Ping AI Accelerator
    try:
        result = subprocess.run(
            ['ping', '-n', '2', AI_ACCELERATOR_IP] if sys.platform == 'win32' else ['ping', '-c', '2', AI_ACCELERATOR_IP],
            capture_output=True,
            timeout=5
        )
        if result.returncode == 0:
            print(f"✅ AI Accelerator ({AI_ACCELERATOR_IP}) raggiungibile")
            results['ai_accelerator'] = True
        else:
            print(f"❌ AI Accelerator ({AI_ACCELERATOR_IP}) NON raggiungibile")
            results['ai_accelerator'] = False
    except Exception as e:
        print(f"⚠️  Ping AI Accelerator fallito: {e}")
        results['ai_accelerator'] = None
    
    # Ping Robot
    try:
        result = subprocess.run(
            ['ping', '-n', '2', ROBOT_IP] if sys.platform == 'win32' else ['ping', '-c', '2', ROBOT_IP],
            capture_output=True,
            timeout=5
        )
        if result.returncode == 0:
            print(f"✅ Robot ({ROBOT_IP}) raggiungibile")
            results['robot'] = True
        else:
            print(f"❌ Robot ({ROBOT_IP}) NON raggiungibile")
            results['robot'] = False
    except Exception as e:
        print(f"⚠️  Ping Robot fallito: {e}")
        results['robot'] = None
    
    return results

def test_web_interface():
    """Test web interface"""
    print("\n" + "=" * 80)
    print("3. VERIFICA WEB INTERFACE")
    print("=" * 80)
    
    try:
        response = requests.get(f"{WEB_URL}/", timeout=5)
        if response.status_code == 200:
            print(f"✅ Web Interface raggiungibile: {WEB_URL}")
            
            # Verifica contenuto
            if "UR Remote Control" in response.text:
                print("✅ Pagina HTML corretta")
            if "Robot Status" in response.text:
                print("✅ Pannello Robot Status presente")
            if "ROS2 Monitor" in response.text:
                print("✅ Pannello ROS2 Monitor presente")
            
            return True
        else:
            print(f"❌ Web Interface errore: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print(f"❌ Web Interface NON raggiungibile: {WEB_URL}")
        print("   La web interface potrebbe non essere avviata")
        print("   Avvia con:")
        print(f"   ssh lab@{AI_ACCELERATOR_IP}")
        print("   cd ~/MekoAiAccelerator")
        print("   export UR_ROBOT_IP=192.168.10.194")
        print(f"   export WEB_PORT={WEB_PORT}")
        print("   python3 -m remote_ur_control.web_interface")
        return False
    except Exception as e:
        print(f"❌ Errore web interface: {e}")
        return False

def test_api_status():
    """Test API status"""
    print("\n" + "=" * 80)
    print("4. VERIFICA API /api/status")
    print("=" * 80)
    
    try:
        response = requests.get(f"{WEB_URL}/api/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ API /api/status OK")
            print(f"   ROS2 Available: {data.get('data', {}).get('ros2_available', False)}")
            if 'ros2_bridge' in data.get('data', {}):
                bridge = data['data']['ros2_bridge']
                print(f"   ROS2 Initialized: {bridge.get('ros_initialized', False)}")
                print(f"   Publish Loop: {bridge.get('publish_loop_running', False)}")
            return True
        else:
            print(f"❌ API /api/status errore: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Errore API /api/status: {e}")
        return False

def test_api_robot_status():
    """Test API robot status"""
    print("\n" + "=" * 80)
    print("5. VERIFICA API /api/robot_status")
    print("=" * 80)
    
    try:
        response = requests.get(f"{WEB_URL}/api/robot_status", timeout=10)
        if response.status_code == 200:
            data = response.json()
            print("✅ API /api/robot_status OK")
            
            if 'data' in data:
                # Dashboard
                dashboard = data['data'].get('dashboard', {})
                if 'error' in dashboard:
                    print(f"   ⚠️  Dashboard Error: {dashboard['error']}")
                else:
                    print(f"   Robot Mode: {dashboard.get('robotmode', 'unknown')}")
                    print(f"   Safety Mode: {dashboard.get('safetymode', 'unknown')}")
                    print(f"   Program State: {dashboard.get('programState', 'unknown')}")
                    print(f"   Remote Control: {dashboard.get('remote_control', 'unknown')}")
                
                # RTDE
                rtde = data['data'].get('rtde', {})
                if 'error' in rtde:
                    print(f"   ⚠️  RTDE Error: {rtde['error']}")
                else:
                    if 'joints' in rtde:
                        joints = rtde['joints']
                        print(f"   Joints: [{', '.join(f'{j:.4f}' for j in joints[:3])}...]")
                    if 'tcp_pose' in rtde:
                        tcp = rtde['tcp_pose']
                        print(f"   TCP Pose: [{', '.join(f'{p:.4f}' for p in tcp[:3])}...]")
            
            return True
        else:
            print(f"❌ API /api/robot_status errore: {response.status_code}")
            print(f"   Response: {response.text[:200]}")
            return False
    except requests.exceptions.Timeout:
        print("❌ Timeout - Il robot potrebbe non rispondere")
        return False
    except Exception as e:
        print(f"❌ Errore API /api/robot_status: {e}")
        return False

def test_robot_ports():
    """Test porte robot"""
    print("\n" + "=" * 80)
    print("6. VERIFICA PORTE ROBOT")
    print("=" * 80)
    
    ports = {
        30001: "Primary Interface",
        30002: "URScript Port",
        30004: "RTDE",
        29999: "Dashboard Server"
    }
    
    results = {}
    for port, name in ports.items():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((ROBOT_IP, port))
            sock.close()
            if result == 0:
                print(f"✅ Porta {port} ({name}) APERTA")
                results[port] = True
            else:
                print(f"❌ Porta {port} ({name}) CHIUSA")
                results[port] = False
        except Exception as e:
            print(f"⚠️  Errore test porta {port}: {e}")
            results[port] = None
    
    return results

def main():
    """Esegue tutti i test"""
    print("\n" + "=" * 80)
    print("VERIFICA COMPLETA SISTEMA")
    print("=" * 80)
    print(f"AI Accelerator: {AI_ACCELERATOR_IP}")
    print(f"Robot: {ROBOT_IP}")
    print(f"Web Interface: {WEB_URL}")
    print("=" * 80)
    
    results = {}
    
    # Test 1: SSH
    results['ssh'] = test_ssh_connection()
    
    # Test 2: Ping
    ping_results = test_ping()
    results['ping'] = ping_results
    
    # Test 3: Web Interface
    results['web_interface'] = test_web_interface()
    
    # Test 4: API Status
    if results['web_interface']:
        results['api_status'] = test_api_status()
    else:
        results['api_status'] = False
        print("\n⚠️  Saltato test API (web interface non disponibile)")
    
    # Test 5: API Robot Status
    if results['web_interface']:
        results['api_robot_status'] = test_api_robot_status()
    else:
        results['api_robot_status'] = False
        print("\n⚠️  Saltato test API robot status (web interface non disponibile)")
    
    # Test 6: Porte Robot
    results['robot_ports'] = test_robot_ports()
    
    # Riepilogo
    print("\n" + "=" * 80)
    print("RIEPILOGO")
    print("=" * 80)
    
    for test, result in results.items():
        if isinstance(result, dict):
            print(f"\n{test}:")
            for key, value in result.items():
                status = "✅" if value is True else "❌" if value is False else "⚠️"
                print(f"  {status} {key}")
        else:
            status = "✅" if result is True else "❌" if result is False else "⚠️"
            print(f"{status} {test}")
    
    # Conclusioni
    print("\n" + "=" * 80)
    print("CONCLUSIONI")
    print("=" * 80)
    
    if results.get('web_interface'):
        print("✅ Web Interface funzionante!")
        print(f"   Apri browser su: {WEB_URL}")
        print("   Dovresti vedere:")
        print("     - ROS2 Monitor")
        print("     - Robot Status (con joints e TCP pose)")
        print("     - Joystick e controlli")
    else:
        print("❌ Web Interface NON disponibile")
        print("\nPer avviarla:")
        print(f"  ssh lab@{AI_ACCELERATOR_IP}")
        print("  cd ~/MekoAiAccelerator")
        print(f"  export UR_ROBOT_IP={ROBOT_IP}")
        print(f"  export WEB_PORT={WEB_PORT}")
        print("  python3 -m remote_ur_control.web_interface")
    
    if results.get('api_robot_status'):
        print("\n✅ API Robot Status funzionante!")
        print("   Lo stato del robot viene letto correttamente")
    else:
        print("\n⚠️  API Robot Status non disponibile")
        print("   Verifica che:")
        print("     - Robot sia acceso")
        print("     - Robot sia raggiungibile")
        print("     - ur_rtde sia installato")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())










