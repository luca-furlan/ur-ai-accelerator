#!/usr/bin/env python3
"""
Test Camera Orbbec - verifica disponibilità e funzionamento
"""

import subprocess
import sys
import os

def check_ros2_orbbec_topics() -> list:
    """Verifica topics Orbbec in ROS2"""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            topics = [t.strip() for t in result.stdout.split('\n') if t.strip()]
            orbbec_topics = [t for t in topics if 'camera' in t.lower() or 'orbbec' in t.lower()]
            return orbbec_topics
        return []
    except:
        return []

def check_orbbec_ros2_package() -> bool:
    """Verifica se pacchetto ROS2 Orbbec è installato"""
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            packages = result.stdout.lower()
            return 'orbbec' in packages
        return False
    except:
        return False

def test_orbbec_sdk_python() -> bool:
    """Test import OrbbecSDK Python"""
    try:
        import orbbecsdk
        return True
    except ImportError:
        return False

def test_camera_connection() -> bool:
    """Test connessione camera (via ROS2 topic)"""
    topics = check_ros2_orbbec_topics()
    if not topics:
        return False
    
    # Prova a leggere un topic
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', topics[0], '--once'],
            capture_output=True,
            timeout=3
        )
        return result.returncode == 0 and len(result.stdout) > 0
    except:
        return False

def main():
    print("=" * 60)
    print("TEST CAMERA ORBBEC")
    print("=" * 60)
    
    # Verifica pacchetto ROS2
    print("\n1. Verifica pacchetto ROS2 Orbbec...")
    if check_orbbec_ros2_package():
        print("✅ Pacchetto ROS2 Orbbec installato")
    else:
        print("⚠️ Pacchetto ROS2 Orbbec non trovato")
    
    # Verifica topics
    print("\n2. Verifica topics Orbbec...")
    topics = check_ros2_orbbec_topics()
    if topics:
        print(f"✅ Topics Orbbec disponibili: {len(topics)}")
        print("   Topics principali:")
        for topic in topics[:10]:
            print(f"     - {topic}")
    else:
        print("⚠️ Nessun topic Orbbec rilevato (camera non avviata?)")
    
    # Test connessione
    print("\n3. Test connessione camera...")
    if test_camera_connection():
        print("✅ Camera connessa e funzionante")
    else:
        print("⚠️ Camera non connessa o non pubblica dati")
    
    # Test SDK Python
    print("\n4. Verifica OrbbecSDK Python...")
    if test_orbbec_sdk_python():
        print("✅ OrbbecSDK Python installato")
    else:
        print("⚠️ OrbbecSDK Python non installato (opzionale)")
    
    print("\n" + "=" * 60)
    print("NOTA: Per avviare la camera Orbbec:")
    print("  ros2 launch orbbec_camera gemini_330_series.launch.py")
    print("=" * 60)

if __name__ == '__main__':
    main()





