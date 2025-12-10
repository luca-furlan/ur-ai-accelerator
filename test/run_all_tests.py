#!/usr/bin/env python3
"""
Script Master - Esegue tutti i test del sistema
"""

import os
import sys
import subprocess
import time
from datetime import datetime

# Path test directory
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(TEST_DIR)

# Colori
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_header(text: str):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text.center(80)}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.RESET}\n")

def run_test(test_file: str, description: str) -> bool:
    """Esegue un test e ritorna successo"""
    print(f"\n{Colors.BOLD}▶ {description}{Colors.RESET}")
    print(f"   Esecuzione: {test_file}")
    
    test_path = os.path.join(TEST_DIR, test_file)
    if not os.path.exists(test_path):
        print(f"{Colors.RED}❌ File test non trovato: {test_path}{Colors.RESET}")
        return False
    
    try:
        result = subprocess.run(
            [sys.executable, test_path],
            cwd=PROJECT_ROOT,
            timeout=60,
            capture_output=False
        )
        success = result.returncode == 0
        if success:
            print(f"{Colors.GREEN}✅ Test completato{Colors.RESET}")
        else:
            print(f"{Colors.YELLOW}⚠️ Test completato con warning/errori{Colors.RESET}")
        return success
    except subprocess.TimeoutExpired:
        print(f"{Colors.RED}❌ Test timeout{Colors.RESET}")
        return False
    except Exception as e:
        print(f"{Colors.RED}❌ Errore esecuzione: {e}{Colors.RESET}")
        return False

def main():
    print_header("ESECUZIONE TUTTI I TEST DEL SISTEMA")
    print(f"Data/Ora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Directory test: {TEST_DIR}")
    print(f"Project root: {PROJECT_ROOT}")
    
    # Lista test da eseguire
    tests = [
        ('test_connettivita_robot.py', 'Test Connettività Robot'),
        ('test_ros2_driver.py', 'Test ROS2 Driver UR'),
        ('test_camera_orbbec.py', 'Test Camera Orbbec'),
        ('test_mujoco.py', 'Test MuJoCo'),
        ('test_ai_components.py', 'Test Componenti AI'),
        ('test_web_interface.py', 'Test Web Interface'),
    ]
    
    results = {}
    total_tests = len(tests)
    passed_tests = 0
    
    for test_file, description in tests:
        success = run_test(test_file, description)
        results[description] = success
        if success:
            passed_tests += 1
        time.sleep(1)  # Pausa tra test
    
    # Riepilogo
    print_header("RIEPILOGO TEST")
    print(f"\n{Colors.BOLD}Statistiche:{Colors.RESET}")
    print(f"  {Colors.GREEN}✅ Test passati: {passed_tests}/{total_tests}{Colors.RESET}")
    print(f"  {Colors.RED}❌ Test falliti: {total_tests - passed_tests}/{total_tests}{Colors.RESET}")
    
    print(f"\n{Colors.BOLD}Dettaglio:{Colors.RESET}")
    for description, success in results.items():
        status = f"{Colors.GREEN}✅{Colors.RESET}" if success else f"{Colors.RED}❌{Colors.RESET}"
        print(f"  {status} {description}")
    
    # Test completo sistema
    print_header("TEST COMPLETO SISTEMA")
    print("Esecuzione verifica completa installazione...")
    test_completo = os.path.join(PROJECT_ROOT, 'test_sistema_completo.py')
    if os.path.exists(test_completo):
        subprocess.run([sys.executable, test_completo], cwd=PROJECT_ROOT)
    else:
        print(f"{Colors.YELLOW}⚠️ test_sistema_completo.py non trovato{Colors.RESET}")
    
    print(f"\n{Colors.BOLD}Test completati!{Colors.RESET}\n")

if __name__ == '__main__':
    main()





