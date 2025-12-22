#!/usr/bin/env python3
"""
Script per testare la web interface e verificare che mostri lo stato del robot
"""

import requests
import json
import sys
import time

WEB_URL = "http://192.168.10.191:8081"  # Modifica se necessario

def test_api_status():
    """Test endpoint /api/status"""
    print("=" * 80)
    print("TEST: /api/status (ROS2 Status)")
    print("=" * 80)
    try:
        response = requests.get(f"{WEB_URL}/api/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ Status API OK")
            print(f"   ROS2 Available: {data.get('data', {}).get('ros2_available', False)}")
            if 'ros2_bridge' in data.get('data', {}):
                bridge = data['data']['ros2_bridge']
                print(f"   ROS2 Initialized: {bridge.get('ros_initialized', False)}")
                print(f"   Publish Loop Running: {bridge.get('publish_loop_running', False)}")
            return True
        else:
            print(f"❌ Status API Error: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("❌ Impossibile connettersi alla web interface")
        print(f"   URL: {WEB_URL}")
        print("   Verifica che la web interface sia avviata:")
        print("   python3 -m remote_ur_control.web_interface")
        return False
    except Exception as e:
        print(f"❌ Errore: {e}")
        return False

def test_api_robot_status():
    """Test endpoint /api/robot_status"""
    print("\n" + "=" * 80)
    print("TEST: /api/robot_status (Robot Status)")
    print("=" * 80)
    try:
        response = requests.get(f"{WEB_URL}/api/robot_status", timeout=10)
        if response.status_code == 200:
            data = response.json()
            print("✅ Robot Status API OK")
            
            if 'data' in data:
                # Dashboard status
                dashboard = data['data'].get('dashboard', {})
                if 'error' in dashboard:
                    print(f"   ⚠️  Dashboard Error: {dashboard['error']}")
                else:
                    print(f"   Robot Mode: {dashboard.get('robotmode', 'unknown')}")
                    print(f"   Safety Mode: {dashboard.get('safetymode', 'unknown')}")
                    print(f"   Program State: {dashboard.get('programState', 'unknown')}")
                    print(f"   Remote Control: {dashboard.get('remote_control', 'unknown')}")
                
                # RTDE data
                rtde = data['data'].get('rtde', {})
                if 'error' in rtde:
                    print(f"   ⚠️  RTDE Error: {rtde['error']}")
                else:
                    if 'joints' in rtde:
                        joints = rtde['joints']
                        print(f"   Joints (rad): [{', '.join(f'{j:.4f}' for j in joints)}]")
                    if 'tcp_pose' in rtde:
                        tcp = rtde['tcp_pose']
                        print(f"   TCP Pose: [{', '.join(f'{p:.4f}' for p in tcp)}]")
            
            return True
        else:
            print(f"❌ Robot Status API Error: {response.status_code}")
            print(f"   Response: {response.text}")
            return False
    except requests.exceptions.Timeout:
        print("❌ Timeout - Il robot potrebbe non rispondere")
        return False
    except Exception as e:
        print(f"❌ Errore: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_web_interface():
    """Test pagina principale"""
    print("\n" + "=" * 80)
    print("TEST: Web Interface (Pagina principale)")
    print("=" * 80)
    try:
        response = requests.get(f"{WEB_URL}/", timeout=5)
        if response.status_code == 200:
            print("✅ Web Interface raggiungibile")
            if "UR Remote Control" in response.text:
                print("✅ Pagina HTML corretta")
            if "Robot Status" in response.text:
                print("✅ Pannello Robot Status presente")
            if "ROS2 Monitor" in response.text:
                print("✅ Pannello ROS2 Monitor presente")
            return True
        else:
            print(f"❌ Web Interface Error: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Errore: {e}")
        return False

def main():
    print("\n" + "=" * 80)
    print("TEST WEB INTERFACE - STATO ROBOT")
    print("=" * 80)
    print(f"URL: {WEB_URL}")
    print()
    
    results = {
        "Web Interface": test_web_interface(),
        "API Status": test_api_status(),
        "API Robot Status": test_api_robot_status(),
    }
    
    print("\n" + "=" * 80)
    print("RIEPILOGO")
    print("=" * 80)
    for test, result in results.items():
        status = "✅ OK" if result else "❌ FAIL"
        print(f"{test:30s}: {status}")
    
    all_ok = all(results.values())
    
    if all_ok:
        print("\n✅ TUTTI I TEST PASSATI!")
        print(f"\nApri il browser su: {WEB_URL}")
        print("Dovresti vedere:")
        print("  - Pannello ROS2 Monitor (in alto)")
        print("  - Pannello Robot Status (sotto ROS2 Monitor)")
        print("  - Joystick e controlli (sotto)")
    else:
        print("\n⚠️  ALCUNI TEST FALLITI")
        print("\nPer avviare la web interface:")
        print("  ssh lab@192.168.10.191")
        print("  cd ~/MekoAiAccelerator")
        print("  export UR_ROBOT_IP=192.168.10.194")
        print("  export WEB_PORT=8081")
        print("  python3 -m remote_ur_control.web_interface")
    
    return 0 if all_ok else 1

if __name__ == "__main__":
    sys.exit(main())










