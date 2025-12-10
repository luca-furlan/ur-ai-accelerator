#!/usr/bin/env python3
"""
Test Web Interface - verifica funzionamento
"""

import os
import sys
import subprocess
import time
import socket

# Aggiungi path progetto
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_web_interface_import() -> bool:
    """Test import web interface"""
    try:
        from remote_ur_control import web_interface
        return True
    except ImportError as e:
        print(f"Errore import: {e}")
        return False

def test_ros2_bridge_import() -> bool:
    """Test import ROS2 bridge"""
    try:
        from ros2_bridge_fixed import ROS2Bridge, ROS2_AVAILABLE
        return True
    except ImportError as e:
        print(f"Errore import: {e}")
        return False

def test_ros2_bridge_init() -> bool:
    """Test inizializzazione ROS2 bridge"""
    try:
        from ros2_bridge_fixed import ROS2Bridge
        bridge = ROS2Bridge()
        if bridge.ensure_ros():
            return True
        return False
    except Exception as e:
        print(f"Errore inizializzazione: {e}")
        return False

def test_web_interface_files() -> dict:
    """Verifica presenza file web interface"""
    results = {
        'web_interface_py': False,
        'ros2_bridge_py': False,
        'remote_controller_py': False,
    }
    
    project_root = os.path.expanduser('~/MekoAiAccelerator')
    
    web_interface = os.path.join(project_root, 'remote_ur_control', 'web_interface.py')
    ros2_bridge = os.path.join(project_root, 'ros2_bridge_fixed.py')
    remote_controller = os.path.join(project_root, 'remote_ur_control', 'remote_ur_controller.py')
    
    results['web_interface_py'] = os.path.isfile(web_interface)
    results['ros2_bridge_py'] = os.path.isfile(ros2_bridge)
    results['remote_controller_py'] = os.path.isfile(remote_controller)
    
    return results

def test_port_available(port: int = 8080) -> bool:
    """Test se porta è disponibile"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('localhost', port))
        sock.close()
        return result != 0  # Porta disponibile se non si connette
    except:
        return True

def main():
    print("=" * 60)
    print("TEST WEB INTERFACE")
    print("=" * 60)
    
    # Verifica file
    print("\n1. Verifica file componenti...")
    files = test_web_interface_files()
    if files['web_interface_py']:
        print("✅ web_interface.py presente")
    else:
        print("❌ web_interface.py NON presente")
    
    if files['ros2_bridge_py']:
        print("✅ ros2_bridge_fixed.py presente")
    else:
        print("❌ ros2_bridge_fixed.py NON presente")
    
    if files['remote_controller_py']:
        print("✅ remote_ur_controller.py presente")
    else:
        print("❌ remote_ur_controller.py NON presente")
    
    # Test import
    print("\n2. Test import moduli...")
    if test_web_interface_import():
        print("✅ Web interface importabile")
    else:
        print("❌ Web interface NON importabile")
    
    if test_ros2_bridge_import():
        print("✅ ROS2 bridge importabile")
    else:
        print("❌ ROS2 bridge NON importabile")
    
    # Test inizializzazione ROS2 bridge
    print("\n3. Test inizializzazione ROS2 bridge...")
    if test_ros2_bridge_init():
        print("✅ ROS2 bridge inizializzato")
    else:
        print("⚠️ ROS2 bridge non inizializzato (ROS2 non disponibile?)")
    
    # Test porta
    print("\n4. Verifica porta web (8080)...")
    if test_port_available(8080):
        print("✅ Porta 8080 disponibile")
    else:
        print("⚠️ Porta 8080 occupata (web interface già in esecuzione?)")
    
    print("\n" + "=" * 60)
    print("NOTA: Per avviare web interface:")
    print("  cd ~/MekoAiAccelerator")
    print("  export UR_ROBOT_IP=192.168.10.194")
    print("  export WEB_HOST=0.0.0.0")
    print("  export WEB_PORT=8080")
    print("  python3 -m remote_ur_control.web_interface")
    print("=" * 60)

if __name__ == '__main__':
    main()





