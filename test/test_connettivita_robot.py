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
        # Verifica se è installato ma non importabile (su aarch64)
        import subprocess
        try:
            result = subprocess.run(['pip3', 'list'], capture_output=True, text=True, timeout=5)
            if 'ur-rtde' in result.stdout:
                print("⚠️ ur_rtde installato ma non importabile (probabile problema aarch64)")
                return False
        except:
            pass
        print("⚠️ ur_rtde non installato")
        return False
    except Exception as e:
        error_msg = str(e)
        if 'aarch64' in error_msg.lower() or 'arm' in error_msg.lower():
            print(f"⚠️ ur_rtde errore su aarch64: {e}")
        else:
            print(f"Errore RTDE: {e}")
        return False

def main():
    robot_ip = '192.168.10.194'
    
    print("=" * 60)
    print("TEST CONNETTIVITÀ ROBOT")
    print("=" * 60)
    
    # Test socket
    print(f"\n1. Test connessione socket ({robot_ip}:30002)...")
    socket_ok = test_robot_connection(robot_ip)
    if socket_ok:
        print("✅ Socket connesso")
    else:
        print("❌ Socket NON connesso")
    
    # Test RTDE
    print(f"\n2. Test connessione RTDE ({robot_ip}:30004)...")
    rtde_ok = test_robot_rtde(robot_ip)
    if rtde_ok:
        print("✅ RTDE connesso")
    else:
        print("❌ RTDE NON connesso")
    
    # Riepilogo
    print("\n" + "=" * 60)
    if socket_ok and rtde_ok:
        print("✅ Tutti i test superati!")
    elif socket_ok or rtde_ok:
        print("⚠️ Alcuni test non superati")
    else:
        print("❌ Nessun test superato - robot non raggiungibile?")
    print("=" * 60)

if __name__ == '__main__':
    main()












