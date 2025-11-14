"""
Test movimento diretto con script URScript più espliciti.
Usa servoj per movimento continuo che funziona anche senza programma attivo.
"""

import os
import socket
import sys
import time

def send_script(robot_ip: str, script: str, port: int = 30002):
    """Invia script URScript direttamente."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((robot_ip, port))
    sock.sendall(script.encode('utf-8'))
    sock.close()

def test_servoj(robot_ip: str):
    """Test con servoj - movimento continuo che funziona meglio."""
    print("\n[TEST] ServoJ - movimento continuo...")
    
    # Ottieni posizione corrente
    get_pos_script = (
        "def get_pos():\n"
        "  current = get_actual_joint_positions()\n"
        "  textmsg(\"CURRENT_POS:\", current[0], current[1], current[2], current[3], current[4], current[5])\n"
        "end\n"
        "get_pos()\n"
    )
    
    print("  Lettura posizione corrente...")
    send_script(robot_ip, get_pos_script)
    time.sleep(0.5)
    
    # Movimento con servoj - più diretto
    servoj_script = (
        "def servo_move():\n"
        "  current = get_actual_joint_positions()\n"
        "  target = [current[0] + 0.1, current[1], current[2], current[3], current[4], current[5]]\n"
        "  servoj(target, t=0.5, lookahead_time=0.1, gain=300)\n"
        "end\n"
        "servo_move()\n"
    )
    
    print("  Invio comando servoj (joint 0 + 0.1 rad)...")
    print("  [ATTENZIONE] Il robot dovrebbe muoversi ORA!")
    send_script(robot_ip, servoj_script)
    time.sleep(2)
    
    print("  [OK] Comando inviato")

def test_movej_explicit(robot_ip: str):
    """Test movej con script più esplicito."""
    print("\n[TEST] MoveJ esplicito...")
    
    script = (
        "def explicit_move():\n"
        "  current = get_actual_joint_positions()\n"
        "  target = [current[0] + 0.15, current[1], current[2], current[3], current[4], current[5]]\n"
        "  movej(target, a=0.5, v=0.15, r=0.0)\n"
        "end\n"
        "explicit_move()\n"
    )
    
    print("  Invio comando movej (joint 0 + 0.15 rad)...")
    print("  [ATTENZIONE] Il robot dovrebbe muoversi ORA!")
    send_script(robot_ip, script)
    time.sleep(3)
    
    print("  [OK] Comando inviato")

def check_and_start_program(robot_ip: str):
    """Verifica e avvia il programma se necessario."""
    import socket
    
    print("\n[CHECK] Verifica programma...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((robot_ip, 29999))
    sock.recv(1024)  # Welcome
    
    # Verifica stato
    sock.sendall(b"programState\n")
    time.sleep(0.2)
    state = sock.recv(1024).decode('utf-8').strip()
    print(f"  Program State: {state}")
    
    if "STOPPED" in state or "PAUSED" in state:
        print("  Programma fermo - tentativo di avvio...")
        sock.sendall(b"play\n")
        time.sleep(0.2)
        response = sock.recv(1024).decode('utf-8').strip()
        print(f"  Risposta: {response}")
        time.sleep(2)
    
    sock.close()

def main():
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("=" * 60)
    print("TEST MOVIMENTO DIRETTO")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}\n")
    
    # Verifica e avvia programma
    check_and_start_program(robot_ip)
    
    print("\n" + "=" * 60)
    print("TEST MOVIMENTI")
    print("=" * 60)
    print("\n[ATTENZIONE] Il robot dovrebbe muoversi durante questi test!")
    print("Premere INVIO per iniziare i test...")
    
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("\nTest annullato")
        return 1
    
    # Test 1: ServoJ
    test_servoj(robot_ip)
    time.sleep(1)
    
    # Test 2: MoveJ esplicito
    test_movej_explicit(robot_ip)
    
    print("\n" + "=" * 60)
    print("TEST COMPLETATI")
    print("=" * 60)
    print("\nSe il robot NON si e mosso:")
    print("  1. Verifica sul teach pendant che il programma sia PLAYING")
    print("  2. Controlla che non ci siano errori sul teach pendant")
    print("  3. Verifica che il robot non sia in SAFE STOP")
    print("  4. Prova a premere PLAY manualmente sul teach pendant")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

