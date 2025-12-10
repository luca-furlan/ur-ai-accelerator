#!/usr/bin/env python3
"""
Test connettività e comunicazione con robot UR
"""

import socket
import sys
import time
from typing import Optional

def test_robot_connection(robot_ip: str = '192.168.10.194', port: int = 30002, timeout: int = 3) -> bool:
    """Test connessione socket al robot"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((robot_ip, port))
        sock.close()
        return result == 0
    except Exception as e:
        print(f"Errore connessione: {e}")
        return False

def test_robot_rtde(robot_ip: str = '192.168.10.194') -> bool:
    """Test connessione RTDE al robot"""
    try:
        import ur_rtde
        rtde = ur_rtde.RTDE(robot_ip, 30004)
        rtde.connect()
        # Prova a leggere stato
        state = rtde.receive()
        rtde.disconnect()
        return state is not None
    except ImportError:
        print("⚠️ ur_rtde non installato")
        return False
    except Exception as e:
        print(f"Errore RTDE: {e}")
        return False

def main():
    robot_ip = '192.168.10.194'
    
    print("=" * 60)
    print("TEST CONNETTIVITÀ ROBOT")
    print("=" * 60)
    
    # Test socket
    print(f"\n1. Test connessione socket ({robot_ip}:30002)...")
    if test_robot_connection(robot_ip):
        print("✅ Socket connesso")
    else:
        print("❌ Socket NON connesso")
    
    # Test RTDE
    print(f"\n2. Test connessione RTDE ({robot_ip}:30004)...")
    if test_robot_rtde(robot_ip):
        print("✅ RTDE connesso")
    else:
        print("❌ RTDE NON connesso")
    
    print("\n" + "=" * 60)

if __name__ == '__main__':
    main()





