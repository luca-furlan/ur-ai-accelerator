"""
Test se gli script URScript vengono effettivamente eseguiti dal robot.
Usa textmsg per verificare l'esecuzione.
"""

import os
import socket
import sys
import time

def test_script_reception(robot_ip: str, port: int = 30002):
    """Test se gli script vengono ricevuti ed eseguiti."""
    print("=" * 60)
    print("TEST ESECUZIONE SCRIPT URScript")
    print("=" * 60)
    print(f"Robot IP: {robot_ip}:{port}\n")
    
    # Script di test che fa solo logging
    test_scripts = [
        {
            "name": "Test 1: Script semplice textmsg",
            "script": (
                "def test1():\n"
                '  textmsg("TEST_SCRIPT_1_EXECUTED")\n'
                "end\n"
                "test1()\n"
            )
        },
        {
            "name": "Test 2: Script con movimento piccolo",
            "script": (
                "def test2():\n"
                "  current = get_actual_joint_positions()\n"
                "  target = [current[0] + 0.2, current[1], current[2], current[3], current[4], current[5]]\n"
                '  textmsg("MOVING_JOINT_0_BY_0.2")\n'
                "  movej(target, a=0.5, v=0.2)\n"
                "end\n"
                "test2()\n"
            )
        },
        {
            "name": "Test 3: ServoJ continuo",
            "script": (
                "def test3():\n"
                "  current = get_actual_joint_positions()\n"
                "  target = [current[0] + 0.15, current[1], current[2], current[3], current[4], current[5]]\n"
                '  textmsg("SERVOJ_MOVE")\n'
                "  servoj(target, t=0.5, lookahead_time=0.1, gain=300)\n"
                "end\n"
                "test3()\n"
            )
        }
    ]
    
    for i, test in enumerate(test_scripts, 1):
        print(f"\n[{i}/{len(test_scripts)}] {test['name']}")
        print("-" * 60)
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((robot_ip, port))
            
            print("  Connessione stabilita")
            print(f"  Invio script ({len(test['script'])} bytes)...")
            
            # Mostra script
            print("\n  Script:")
            for line in test['script'].split('\n'):
                print(f"    {line}")
            
            sock.sendall(test['script'].encode('utf-8'))
            print("\n  [OK] Script inviato")
            
            # Attesa breve per vedere se c'è risposta
            sock.settimeout(1.0)
            try:
                response = sock.recv(1024)
                if response:
                    print(f"  Risposta ricevuta: {response.decode('utf-8', errors='ignore')}")
            except socket.timeout:
                print("  Nessuna risposta (normale per URScript port)")
            
            sock.close()
            
            if i > 1:  # Per i test di movimento
                print("\n  [ATTENZIONE] Controlla il robot - dovrebbe muoversi!")
                print("  Attesa 3 secondi...")
                time.sleep(3)
            
        except Exception as e:
            print(f"  [ERR] Errore: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("TEST COMPLETATI")
    print("=" * 60)
    print("\n[NOTA] Gli script sono stati inviati.")
    print("Per verificare se sono stati eseguiti:")
    print("  1. Controlla i log sul teach pendant (Program -> Log)")
    print("  2. Cerca i messaggi: TEST_SCRIPT_1_EXECUTED, MOVING_JOINT_0_BY_0.2, SERVOJ_MOVE")
    print("  3. Se vedi questi messaggi nei log, gli script vengono eseguiti")
    print("  4. Se il robot si muove, i comandi funzionano correttamente")
    print("\nSe NON vedi i messaggi nei log:")
    print("  - Il programma sul teach pendant potrebbe non essere PLAYING")
    print("  - Il robot potrebbe essere in uno stato che blocca l'esecuzione")
    print("  - Verifica sul teach pendant che ci sia un programma attivo")

if __name__ == "__main__":
    robot_ip = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
    
    print("\n[INFO] Questo script inviera script di test al robot.")
    print("Verifica sul teach pendant se i messaggi compaiono nei log.\n")
    
    test_script_reception(robot_ip)

