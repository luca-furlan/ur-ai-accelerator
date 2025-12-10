#!/usr/bin/env python3
"""
Script per verificare e testare il controllo robot in modo interattivo
Con logging dettagliato di ogni operazione
"""

import sys
import time
import logging
from pathlib import Path
from datetime import datetime

# Setup logging
LOG_DIR = Path.home() / "MekoAiAccelerator" / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
LOG_FILE = LOG_DIR / f"test_controllo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(funcName)s: %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)

LOG = logging.getLogger(__name__)

ROBOT_IP = "192.168.10.194"

def test_controllo_senza_urcap():
    """Test controllo senza URCap"""
    LOG.info("=" * 80)
    LOG.info("TEST: Controllo senza URCap (Primary Interface)")
    LOG.info("=" * 80)
    
    try:
        import socket
        
        LOG.info("Connessione a Primary Interface (30001)...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ROBOT_IP, 30001))
        LOG.info("✅ Connesso")
        
        # Test lettura joints
        script = """
def read_test():
    joints = get_actual_joint_positions()
    textmsg("JOINTS: ", joints[0], ", ", joints[1], ", ", joints[2], ", ", joints[3], ", ", joints[4], ", ", joints[5])
end
read_test()
"""
        LOG.info("Invio script lettura joints...")
        sock.send(script.encode() + b"\n")
        time.sleep(1)
        sock.close()
        LOG.info("✅ Script inviato - Controlla Teach Pendant per vedere joints")
        return True
        
    except Exception as e:
        LOG.error(f"❌ Errore: {e}")
        import traceback
        LOG.error(traceback.format_exc())
        return False

def test_rtde_lettura():
    """Test lettura RTDE"""
    LOG.info("\n" + "=" * 80)
    LOG.info("TEST: Lettura RTDE")
    LOG.info("=" * 80)
    
    try:
        import rtde.rtde as rtde
        
        LOG.info("Connessione RTDE...")
        client = rtde.RTDE(ROBOT_IP, 30004)
        client.connect()
        LOG.info("✅ Connesso")
        
        LOG.info("Setup output...")
        client.send_output_setup(["actual_q", "actual_TCP_pose"], [], frequency=10)
        client.send_start()
        LOG.info("✅ Setup completato")
        
        LOG.info("Lettura dati...")
        state = client.receive()
        if state:
            joints = [round(j, 3) for j in state.actual_q]
            tcp = [round(p, 3) for p in state.actual_TCP_pose]
            LOG.info(f"✅ Dati ricevuti:")
            LOG.info(f"   Joints (rad): {joints}")
            LOG.info(f"   TCP pose: {tcp}")
            client.send_pause()
            client.disconnect()
            return True
        else:
            LOG.warning("⚠️  Nessun dato ricevuto")
            client.disconnect()
            return False
            
    except ImportError:
        LOG.error("❌ ur_rtde non installato: pip3 install --user ur-rtde")
        return False
    except Exception as e:
        LOG.error(f"❌ Errore: {e}")
        import traceback
        LOG.error(traceback.format_exc())
        return False

def test_remote_controller():
    """Test Remote UR Controller"""
    LOG.info("\n" + "=" * 80)
    LOG.info("TEST: Remote UR Controller")
    LOG.info("=" * 80)
    
    try:
        sys.path.insert(0, str(Path(__file__).parent))
        from remote_ur_control.remote_ur_controller import RemoteURController
        
        LOG.info("Creazione controller...")
        controller = RemoteURController(ROBOT_IP)
        
        LOG.info("Connessione...")
        controller.connect()
        LOG.info("✅ Connesso")
        
        LOG.info("Test movimento (NON eseguito, solo test connessione)...")
        # Non muoviamo il robot, solo testiamo la connessione
        controller.close()
        LOG.info("✅ Controller funzionante")
        return True
        
    except Exception as e:
        LOG.error(f"❌ Errore: {e}")
        import traceback
        LOG.error(traceback.format_exc())
        return False

def test_movimento_sicuro():
    """Test movimento molto piccolo e sicuro"""
    LOG.info("\n" + "=" * 80)
    LOG.info("TEST: Movimento Sicuro (Richiede conferma)")
    LOG.info("=" * 80)
    
    risposta = input("\n⚠️  ATTENZIONE: Questo muoverà il robot di 5mm in Z.\nContinuare? (s/n): ")
    if risposta.lower() != 's':
        LOG.info("❌ Test annullato")
        return False
    
    try:
        import socket
        
        LOG.info("Connessione...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ROBOT_IP, 30001))
        LOG.info("✅ Connesso")
        
        script = """
def safe_move_test():
    current = get_actual_tcp_pose()
    target = current
    target[2] = target[2] + 0.005  # +5mm in Z
    movel(target, a=0.05, v=0.02)
    sleep(1)
    movel(current, a=0.05, v=0.02)
end
safe_move_test()
"""
        LOG.info("Invio comando movimento...")
        sock.send(script.encode() + b"\n")
        time.sleep(2)
        sock.close()
        LOG.info("✅ Comando inviato - Robot dovrebbe muoversi di 5mm e tornare")
        return True
        
    except Exception as e:
        LOG.error(f"❌ Errore: {e}")
        import traceback
        LOG.error(traceback.format_exc())
        return False

def main():
    """Menu principale"""
    print("\n" + "=" * 80)
    print("VERIFICA E TEST CONTROLLO ROBOT")
    print("=" * 80)
    print(f"Robot IP: {ROBOT_IP}")
    print(f"Log file: {LOG_FILE}")
    print("=" * 80)
    
    tests = [
        ("1", "Test controllo senza URCap (Primary Interface)", test_controllo_senza_urcap),
        ("2", "Test lettura RTDE", test_rtde_lettura),
        ("3", "Test Remote UR Controller", test_remote_controller),
        ("4", "Test movimento sicuro (5mm) - RICHIEDE CONFERMA", test_movimento_sicuro),
        ("5", "Esegui tutti i test (tranne movimento)", None),
    ]
    
    print("\nTest disponibili:")
    for num, desc, _ in tests:
        print(f"  {num}. {desc}")
    print("  0. Esci")
    
    scelta = input("\nScelta: ").strip()
    
    if scelta == "0":
        print("Uscita")
        return
    
    if scelta == "5":
        # Esegui tutti tranne movimento
        for num, desc, func in tests[:3]:
            if func:
                func()
        return
    
    for num, desc, func in tests:
        if num == scelta and func:
            func()
            break
    else:
        print("Scelta non valida")
    
    print(f"\n✅ Log completo salvato in: {LOG_FILE}")

if __name__ == "__main__":
    main()




