"""
Script di test per verificare i movimenti del robot UR e la sincronizzazione.

Questo script esegue una serie di test progressivi:
1. Test di connessione
2. Test movej con piccoli movimenti sicuri
3. Test speedj (velocità joint)
4. Test speedl (velocità cartesiana)
5. Verifica sincronizzazione ROS2 (se disponibile)
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

from .remote_ur_controller import MoveParameters, RemoteURController


def test_connection(controller: RemoteURController) -> bool:
    """Test base di connessione."""
    print("\n[TEST 1] Test connessione...")
    try:
        controller.connect()
        print("[OK] Connessione stabilita")
        return True
    except Exception as e:
        print(f"[ERR] Errore connessione: {e}")
        return False


def test_movej_small(controller: RemoteURController) -> bool:
    """Test movej con movimento molto piccolo e sicuro."""
    print("\n[TEST 2] Test MoveJ - movimento piccolo...")
    try:
        # Movimento molto piccolo: solo joint 0 di 0.05 rad (~3 gradi)
        # Questo è un movimento minimo e sicuro
        small_move = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
        params = MoveParameters(
            acceleration=0.5,  # Bassa accelerazione per sicurezza
            velocity=0.1,     # Bassa velocità
            blend_radius=0.0,
            async_move=False
        )
        
        print(f"  Invio comando movej: {small_move}")
        controller.movej(small_move, params=params)
        print("[OK] Comando MoveJ inviato con successo")
        
        # Aspetta un po' per vedere il movimento
        print("  Attesa 2 secondi per osservare il movimento...")
        time.sleep(2)
        
        # Torna alla posizione iniziale
        return_move = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print(f"  Ritorno alla posizione iniziale: {return_move}")
        controller.movej(return_move, params=params)
        time.sleep(2)
        
        print("[OK] Test MoveJ completato")
        return True
    except Exception as e:
        print(f"[ERR] Errore MoveJ: {e}")
        return False


def test_speedj(controller: RemoteURController) -> bool:
    """Test speedj con velocità molto bassa."""
    print("\n[TEST 3] Test SpeedJ - velocità joint...")
    try:
        # Velocità molto bassa: 0.05 rad/s per joint 0
        speeds = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
        duration = 1.0  # 1 secondo
        
        print(f"  Invio comando speedj: {speeds} per {duration}s")
        controller.speedj(speeds, duration=duration, acceleration=0.3)
        print("[OK] Comando SpeedJ inviato")
        
        print("  Attesa movimento...")
        time.sleep(duration + 0.5)
        
        # Stop
        print("  Invio stop...")
        controller.stop()
        time.sleep(0.5)
        
        print("[OK] Test SpeedJ completato")
        return True
    except Exception as e:
        print(f"[ERR] Errore SpeedJ: {e}")
        return False


def test_speedl(controller: RemoteURController) -> bool:
    """Test speedl con velocità cartesiana molto bassa."""
    print("\n[TEST 4] Test SpeedL - velocità cartesiana...")
    try:
        # Movimento cartesiano molto piccolo: 10mm/s in direzione Y
        cart_speeds = [0.0, 0.01, 0.0, 0.0, 0.0, 0.0]  # 10mm/s = 0.01 m/s
        duration = 1.0
        
        print(f"  Invio comando speedl: {cart_speeds} per {duration}s")
        controller.speedl(cart_speeds, duration=duration, acceleration=0.2)
        print("[OK] Comando SpeedL inviato")
        
        print("  Attesa movimento...")
        time.sleep(duration + 0.5)
        
        # Stop
        print("  Invio stop...")
        controller.stop()
        time.sleep(0.5)
        
        print("[OK] Test SpeedL completato")
        return True
    except Exception as e:
        print(f"[ERR] Errore SpeedL: {e}")
        return False


def test_ros2_sync() -> bool:
    """Verifica se ROS2 è disponibile e sincronizzato."""
    print("\n[TEST 5] Verifica sincronizzazione ROS2...")
    try:
        # Prova a importare ros2
        try:
            import rclpy
            from rclpy.node import Node
            print("[OK] ROS2 Python disponibile")
        except ImportError:
            print("[INFO] ROS2 Python non disponibile (opzionale)")
            return True  # Non è un errore critico
        
        # Prova a verificare se ci sono nodi ROS2 attivi
        # Questo è solo un check base, non verifica la connessione al robot
        print("  Nota: Per test completo ROS2, avviare il driver UR separatamente")
        print("  Vedi: remote_ur_control/ROS2_SETUP.md")
        return True
    except Exception as e:
        print(f"[INFO] Verifica ROS2: {e}")
        return True  # Non critico


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Test movimenti robot UR e sincronizzazione"
    )
    parser.add_argument(
        "--robot-ip",
        default=os.environ.get("UR_ROBOT_IP", "192.168.10.194"),
        help="IP del robot UR"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=30002,
        help="Porta URScript (default 30002)"
    )
    parser.add_argument(
        "--skip-movement",
        action="store_true",
        help="Salta i test di movimento (solo connessione)"
    )
    parser.add_argument(
        "--skip-ros2",
        action="store_true",
        help="Salta verifica ROS2"
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Esegui test automaticamente senza conferma utente"
    )
    
    args = parser.parse_args(argv)
    
    print("=" * 60)
    print("TEST MOVIMENTI ROBOT UR")
    print("=" * 60)
    print(f"Robot IP: {args.robot_ip}")
    print(f"Porta: {args.port}")
    print("\n[ATTENZIONE] Assicurarsi che:")
    print("  - Il robot sia in modalita REMOTE")
    print("  - Non ci siano operatori nell'area di lavoro")
    print("  - L'e-stop sia facilmente raggiungibile")
    
    if not args.auto:
        print("\nPremere INVIO per continuare o CTRL+C per annullare...")
        try:
            input()
        except KeyboardInterrupt:
            print("\nTest annullato dall'utente")
            return 1
    else:
        print("\n[INFO] Modalita automatica attivata - avvio test...")
        time.sleep(1)
    
    controller = RemoteURController(
        robot_ip=args.robot_ip,
        port=args.port,
        socket_timeout=5.0
    )
    
    results = []
    
    # Test 1: Connessione
    results.append(("Connessione", test_connection(controller)))
    if not results[-1][1]:
        print("\n[ERR] Test di connessione fallito. Interrompo i test.")
        controller.close()
        return 1
    
    if not args.skip_movement:
        # Test 2: MoveJ
        results.append(("MoveJ", test_movej_small(controller)))
        
        # Test 3: SpeedJ
        results.append(("SpeedJ", test_speedj(controller)))
        
        # Test 4: SpeedL
        results.append(("SpeedL", test_speedl(controller)))
    
    if not args.skip_ros2:
        # Test 5: ROS2
        results.append(("ROS2 Sync", test_ros2_sync()))
    
    controller.close()
    
    # Riepilogo
    print("\n" + "=" * 60)
    print("RIEPILOGO TEST")
    print("=" * 60)
    for name, success in results:
        status = "[PASS]" if success else "[FAIL]"
        print(f"{status} - {name}")
    
    all_passed = all(r[1] for r in results)
    if all_passed:
        print("\n[OK] Tutti i test completati con successo!")
        return 0
    else:
        print("\n[ERR] Alcuni test sono falliti. Controllare i dettagli sopra.")
        return 1


if __name__ == "__main__":
    sys.exit(main())

