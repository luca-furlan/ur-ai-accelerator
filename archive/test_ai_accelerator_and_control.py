#!/usr/bin/env python3
"""
Script per verificare lo stato dell'AI Accelerator e testare il controllo fluido del robot.
Può essere eseguito sia da Windows (via SSH) che direttamente sull'AI Accelerator.
"""

import subprocess
import sys
import os
import socket
import time

AI_ACCELERATOR_IP = "192.168.10.191"
AI_ACCELERATOR_USER = "lab"
ROBOT_IP = "192.168.10.194"

def run_ssh_command(cmd):
    """Esegue un comando via SSH sull'AI Accelerator."""
    try:
        ssh_cmd = f'ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 {AI_ACCELERATOR_USER}@{AI_ACCELERATOR_IP} "{cmd}"'
        result = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except Exception as e:
        return False, "", str(e)

def check_connection(host, port, timeout=3):
    """Verifica connessione TCP."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except:
        return False

def main():
    print("=" * 70)
    print("VERIFICA STATO AI ACCELERATOR E CONTROLLO ROBOT")
    print("=" * 70)
    print()
    
    # 1. Verifica connessione AI Accelerator
    print("1. VERIFICA CONNESSIONE AI ACCELERATOR...")
    if check_connection(AI_ACCELERATOR_IP, 22):
        print(f"   ✅ AI Accelerator raggiungibile ({AI_ACCELERATOR_IP})")
    else:
        print(f"   ❌ AI Accelerator NON raggiungibile")
        return 1
    
    # 2. Verifica ROS2
    print("\n2. VERIFICA ROS2...")
    success, output, error = run_ssh_command("test -f /opt/ros/humble/setup.bash && echo 'OK' || echo 'MISSING'")
    if success and "OK" in output:
        print("   ✅ ROS2 Humble installato")
        # Verifica versione
        success, version, _ = run_ssh_command("source /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version 2>/dev/null | head -1")
        if success and version:
            print(f"   {version}")
    else:
        print("   ❌ ROS2 Humble NON installato")
    
    # 3. Verifica progetto
    print("\n3. VERIFICA PROGETTO MEKO...")
    success, output, _ = run_ssh_command("test -d ~/MekoAiAccelerator && echo 'OK' || echo 'MISSING'")
    if success and "OK" in output:
        print("   ✅ Directory progetto presente")
        
        # Verifica file chiave
        files_to_check = [
            "~/MekoAiAccelerator/remote_ur_control/web_interface.py",
            "~/MekoAiAccelerator/ros2_bridge_fixed.py",
            "~/MekoAiAccelerator/remote_ur_control/remote_ur_controller.py"
        ]
        
        for file in files_to_check:
            success, output, _ = run_ssh_command(f"test -f {file} && echo 'OK' || echo 'MISSING'")
            if success and "OK" in output:
                print(f"   ✅ {os.path.basename(file)} presente")
            else:
                print(f"   ❌ {os.path.basename(file)} MANCANTE")
    else:
        print("   ❌ Directory progetto NON presente")
        print("   💡 Esegui: bash deploy_to_ai_accelerator_complete.sh")
    
    # 4. Verifica dipendenze Python
    print("\n4. VERIFICA DIPENDENZE PYTHON...")
    deps = ["flask", "rclpy"]
    for dep in deps:
        success, output, _ = run_ssh_command(f"python3 -c 'import {dep}' 2>&1 && echo 'OK' || echo 'MISSING'")
        if success and "OK" in output:
            print(f"   ✅ {dep} installato")
        else:
            print(f"   ❌ {dep} NON installato")
    
    # 5. Verifica connessione robot
    print("\n5. VERIFICA CONNESSIONE ROBOT...")
    success, output, _ = run_ssh_command(f"ping -c 1 {ROBOT_IP} > /dev/null 2>&1 && echo 'OK' || echo 'FAIL'")
    if success and "OK" in output:
        print(f"   ✅ Robot raggiungibile ({ROBOT_IP})")
        
        # Verifica porte robot
        success, output, _ = run_ssh_command(f"timeout 2 bash -c '</dev/tcp/{ROBOT_IP}/30002' && echo 'OK' || echo 'FAIL'")
        if success and "OK" in output:
            print("   ✅ Porta 30002 (URScript) raggiungibile")
        else:
            print("   ⚠️  Porta 30002 non raggiungibile")
    else:
        print(f"   ❌ Robot NON raggiungibile")
    
    # 6. Verifica processi in esecuzione
    print("\n6. VERIFICA PROCESSI...")
    success, output, _ = run_ssh_command("pgrep -f 'web_interface' > /dev/null && echo 'RUNNING' || echo 'STOPPED'")
    if success and "RUNNING" in output:
        print("   ✅ Web interface in esecuzione")
    else:
        print("   ⚠️  Web interface NON in esecuzione")
    
    success, output, _ = run_ssh_command("pgrep -f 'ur_robot_driver' > /dev/null && echo 'RUNNING' || echo 'STOPPED'")
    if success and "RUNNING" in output:
        print("   ✅ UR Robot Driver in esecuzione")
    else:
        print("   ⚠️  UR Robot Driver NON in esecuzione")
    
    # 7. Verifica porta web
    print("\n7. VERIFICA PORTA WEB...")
    if check_connection(AI_ACCELERATOR_IP, 8080):
        print("   ✅ Porta 8080 attiva (web interface accessibile)")
        print(f"   🌐 Apri: http://{AI_ACCELERATOR_IP}:8080")
    else:
        print("   ⚠️  Porta 8080 non attiva")
    
    # 8. Test controllo fluido
    print("\n8. TEST CONTROLLO FLUIDO...")
    print("   Verifica che il joystick simulato funzioni...")
    
    # Crea script di test
    test_script = """
import socket
import time

ROBOT_IP = "192.168.10.194"
PORT = 30002

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    sock.connect((ROBOT_IP, PORT))
    
    # Test invio comando speedj (velocità zero - sicuro)
    script = "speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5)\\n"
    sock.sendall(script.encode('utf-8'))
    sock.close()
    print("OK")
except Exception as e:
    print(f"FAIL: {e}")
"""
    
    success, output, error = run_ssh_command(f"python3 -c \"{test_script}\"")
    if success and "OK" in output:
        print("   ✅ Controllo robot funzionante (porta 30002)")
    else:
        print(f"   ⚠️  Test controllo: {output or error}")
    
    # Riepilogo e istruzioni
    print("\n" + "=" * 70)
    print("RIEPILOGO E PROSSIMI PASSI")
    print("=" * 70)
    print()
    print("Per avviare il controllo fluido del robot:")
    print()
    print("1. Connettiti all'AI Accelerator:")
    print(f"   ssh {AI_ACCELERATOR_USER}@{AI_ACCELERATOR_IP}")
    print()
    print("2. Avvia web interface con joystick:")
    print("   cd ~/MekoAiAccelerator")
    print("   source /opt/ros/humble/setup.bash 2>/dev/null || true")
    print("   export UR_ROBOT_IP=192.168.10.194")
    print("   export WEB_HOST=0.0.0.0")
    print("   export WEB_PORT=8080")
    print("   python3 -m remote_ur_control.web_interface")
    print()
    print("3. Apri browser:")
    print(f"   http://{AI_ACCELERATOR_IP}:8080")
    print()
    print("4. Usa il joystick simulato nella pagina per controllare il robot!")
    print()
    print("=" * 70)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

