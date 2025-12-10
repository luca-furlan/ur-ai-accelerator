#!/usr/bin/env python3
"""
Quick Check Sistema - Verifica rapida componenti principali
"""

import subprocess
import sys
import os

def check_command(cmd: list, timeout: int = 3) -> bool:
    """Verifica se comando esiste ed eseguibile"""
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=timeout)
        return result.returncode == 0
    except:
        return False

def check_python_package(pkg: str) -> bool:
    """Verifica se pacchetto Python è installato"""
    try:
        __import__(pkg)
        return True
    except:
        return False

def main():
    print("=" * 60)
    print("QUICK CHECK SISTEMA")
    print("=" * 60)
    
    checks = {
        'ROS2': check_command(['ros2', '--version']),
        'Python 3.10+': sys.version_info >= (3, 10),
        'ur_rtde': check_python_package('ur_rtde'),
        'MuJoCo': check_python_package('mujoco'),
        'OpenCV': check_python_package('cv2'),
        'YOLOv8': check_python_package('ultralytics'),
        'Open3D': check_python_package('open3d'),
    }
    
    print("\nComponenti principali:\n")
    for name, status in checks.items():
        status_str = "✅" if status else "❌"
        print(f"  {status_str} {name}")
    
    # Verifica file progetto
    print("\nFile progetto:\n")
    project_files = {
        'Web Interface': os.path.exists(os.path.expanduser('~/MekoAiAccelerator/remote_ur_control/web_interface.py')),
        'ROS2 Bridge': os.path.exists(os.path.expanduser('~/MekoAiAccelerator/ros2_bridge_fixed.py')),
        'Test Sistema': os.path.exists(os.path.expanduser('~/MekoAiAccelerator/test_sistema_completo.py')),
    }
    
    for name, status in project_files.items():
        status_str = "✅" if status else "❌"
        print(f"  {status_str} {name}")
    
    # ROS2 topics (se disponibile)
    if checks['ROS2']:
        print("\nROS2 Runtime:\n")
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, timeout=3)
            if result.returncode == 0:
                topics = [t.strip() for t in result.stdout.decode().split('\n') if t.strip()]
                print(f"  ✅ Topics disponibili: {len(topics)}")
                ur_topics = [t for t in topics if 'ur' in t.lower() or 'joint' in t.lower()]
                if ur_topics:
                    print(f"  ✅ Topics UR: {len(ur_topics)}")
                else:
                    print(f"  ⚠️  Nessun topic UR (driver non avviato?)")
            else:
                print(f"  ⚠️  ROS2 daemon non avviato?")
        except:
            print(f"  ⚠️  Impossibile verificare topics")
    
    print("\n" + "=" * 60)
    print("Per verifica completa: python3 test_sistema_completo.py")
    print("=" * 60)

if __name__ == '__main__':
    main()





