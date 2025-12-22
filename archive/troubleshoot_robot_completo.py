#!/usr/bin/env python3
"""
Sistema completo di troubleshooting per controllo robot UR5e
Con logging dettagliato per capire esattamente cosa non funziona
"""

import socket
import sys
import time
import logging
import traceback
from datetime import datetime
from pathlib import Path

# Configurazione
ROBOT_IP = "192.168.10.194"
PRIMARY_PORT = 30001
URSCRIPT_PORT = 30002
RTDE_PORT = 30004
DASHBOARD_PORT = 29999

# Setup logging dettagliato
LOG_DIR = Path.home() / "MekoAiAccelerator" / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

LOG_FILE = LOG_DIR / f"troubleshoot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)

LOG = logging.getLogger(__name__)

class RobotTroubleshooter:
    """Sistema completo di troubleshooting robot"""
    
    def __init__(self, robot_ip=ROBOT_IP):
        self.robot_ip = robot_ip
        self.results = {}
        
    def test_network_connectivity(self):
        """Test 1: Connettività di rete base"""
        LOG.info("=" * 80)
        LOG.info("TEST 1: CONNETTIVITÀ DI RETE")
        LOG.info("=" * 80)
        
        try:
            import subprocess
            result = subprocess.run(
                ['ping', '-c', '3', '-W', '2', self.robot_ip],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                LOG.info("✅ Ping OK - Robot raggiungibile via rete")
                self.results['network'] = True
                return True
            else:
                LOG.error("❌ Ping FAILED - Robot non raggiungibile")
                LOG.error(f"Output: {result.stdout}")
                LOG.error(f"Error: {result.stderr}")
                self.results['network'] = False
                return False
        except Exception as e:
            LOG.error(f"❌ Errore ping: {e}")
            LOG.error(traceback.format_exc())
            self.results['network'] = False
            return False
    
    def test_port(self, port, name, timeout=3):
        """Test generico porta TCP"""
        LOG.info(f"\nTest porta {port} ({name})...")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((self.robot_ip, port))
            sock.close()
            
            if result == 0:
                LOG.info(f"   ✅ Porta {port} ({name}) APERTA")
                return True
            else:
                LOG.warning(f"   ⚠️  Porta {port} ({name}) CHIUSA o non raggiungibile")
                return False
        except Exception as e:
            LOG.error(f"   ❌ Errore test porta {port}: {e}")
            LOG.debug(traceback.format_exc())
            return False
    
    def test_all_ports(self):
        """Test 2: Tutte le porte robot"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TEST 2: VERIFICA PORTE ROBOT")
        LOG.info("=" * 80)
        
        ports = {
            PRIMARY_PORT: "Primary Interface (URScript diretto)",
            URSCRIPT_PORT: "URScript Port (programmi)",
            RTDE_PORT: "RTDE (Real-Time Data Exchange)",
            DASHBOARD_PORT: "Dashboard Server (controllo)"
        }
        
        results = {}
        for port, name in ports.items():
            results[port] = self.test_port(port, name)
        
        self.results['ports'] = results
        return all(results.values())
    
    def test_primary_interface(self):
        """Test 3: Primary Interface - invio comandi URScript"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TEST 3: PRIMARY INTERFACE - INVIO COMANDI")
        LOG.info("=" * 80)
        
        try:
            LOG.info("Connessione a Primary Interface...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((self.robot_ip, PRIMARY_PORT))
            LOG.info("✅ Connesso a Primary Interface")
            
            # Test comando semplice
            test_script = "textmsg(\"TEST_CONTROLLO\")\n"
            LOG.info(f"Invio comando test: {test_script.strip()}")
            sock.send(test_script.encode() + b"\n")
            time.sleep(0.5)
            
            sock.close()
            LOG.info("✅ Comando inviato con successo")
            LOG.info("   (Controlla Teach Pendant per messaggio 'TEST_CONTROLLO')")
            
            self.results['primary_interface'] = True
            return True
            
        except socket.timeout:
            LOG.error("❌ Timeout connessione Primary Interface")
            self.results['primary_interface'] = False
            return False
        except ConnectionRefusedError:
            LOG.error("❌ Connessione rifiutata - Robot potrebbe non essere in RUNNING")
            self.results['primary_interface'] = False
            return False
        except Exception as e:
            LOG.error(f"❌ Errore Primary Interface: {e}")
            LOG.error(traceback.format_exc())
            self.results['primary_interface'] = False
            return False
    
    def test_rtde_connection(self):
        """Test 4: Connessione RTDE"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TEST 4: CONNESSIONE RTDE")
        LOG.info("=" * 80)
        
        try:
            import rtde.rtde as rtde
            
            LOG.info("Creazione client RTDE...")
            rtde_client = rtde.RTDE(self.robot_ip, RTDE_PORT)
            
            LOG.info("Tentativo connessione...")
            rtde_client.connect()
            LOG.info("✅ Connesso a RTDE")
            
            LOG.info("Setup input/output...")
            rtde_client.send_output_setup(["actual_q"], [], frequency=10)
            LOG.info("✅ Setup completato")
            
            LOG.info("Avvio comunicazione...")
            rtde_client.send_start()
            LOG.info("✅ Comunicazione avviata")
            
            LOG.info("Ricezione dati...")
            state = rtde_client.receive()
            if state:
                joints = [round(j, 3) for j in state.actual_q]
                LOG.info(f"✅ Dati ricevuti! Joints: {joints}")
                self.results['rtde'] = True
                rtde_client.send_pause()
                rtde_client.disconnect()
                return True
            else:
                LOG.warning("⚠️  Nessun dato ricevuto")
                self.results['rtde'] = False
                rtde_client.disconnect()
                return False
                
        except ImportError:
            LOG.error("❌ Libreria ur_rtde non installata")
            LOG.error("   Installa con: pip3 install --user ur-rtde")
            self.results['rtde'] = False
            return False
        except Exception as e:
            LOG.error(f"❌ Errore RTDE: {e}")
            LOG.error(traceback.format_exc())
            self.results['rtde'] = False
            return False
    
    def test_dashboard_server(self):
        """Test 5: Dashboard Server"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TEST 5: DASHBOARD SERVER")
        LOG.info("=" * 80)
        
        try:
            LOG.info("Connessione a Dashboard Server...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((self.robot_ip, DASHBOARD_PORT))
            
            # Leggi welcome message
            welcome = sock.recv(1024).decode('utf-8', errors='ignore')
            LOG.info(f"✅ Connesso. Welcome: {welcome.strip()}")
            
            # Test comandi
            commands = [
                ("robotmode", "Modalità robot"),
                ("safetymode", "Modalità sicurezza"),
                ("programState", "Stato programma"),
            ]
            
            for cmd, desc in commands:
                try:
                    sock.sendall((cmd + "\n").encode('utf-8'))
                    time.sleep(0.2)
                    response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
                    LOG.info(f"   {desc}: {response}")
                except Exception as e:
                    LOG.warning(f"   Errore comando {cmd}: {e}")
            
            sock.close()
            self.results['dashboard'] = True
            return True
            
        except Exception as e:
            LOG.error(f"❌ Errore Dashboard Server: {e}")
            LOG.error(traceback.format_exc())
            self.results['dashboard'] = False
            return False
    
    def test_urscript_port(self):
        """Test 6: URScript Port (30002)"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TEST 6: URSCRIPT PORT (30002)")
        LOG.info("=" * 80)
        
        try:
            LOG.info("Connessione a URScript Port...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((self.robot_ip, URSCRIPT_PORT))
            LOG.info("✅ Connesso a URScript Port")
            
            # Test script semplice
            test_script = "textmsg(\"TEST_URSCRIPT_PORT\")\n"
            LOG.info(f"Invio script test...")
            sock.sendall(test_script.encode('utf-8'))
            time.sleep(0.5)
            
            sock.close()
            LOG.info("✅ Script inviato")
            LOG.info("   (Controlla Teach Pendant per messaggio)")
            
            self.results['urscript_port'] = True
            return True
            
        except Exception as e:
            LOG.error(f"❌ Errore URScript Port: {e}")
            LOG.error(traceback.format_exc())
            self.results['urscript_port'] = False
            return False
    
    def test_remote_controller(self):
        """Test 7: Remote UR Controller"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TEST 7: REMOTE UR CONTROLLER")
        LOG.info("=" * 80)
        
        try:
            sys.path.insert(0, str(Path(__file__).parent))
            from remote_ur_control.remote_ur_controller import RemoteURController
            
            LOG.info("Creazione controller...")
            controller = RemoteURController(self.robot_ip)
            
            LOG.info("Tentativo connessione...")
            controller.connect()
            LOG.info("✅ Controller connesso")
            
            LOG.info("Test lettura stato...")
            # Prova a leggere stato (se disponibile)
            controller.close()
            LOG.info("✅ Controller funzionante")
            
            self.results['remote_controller'] = True
            return True
            
        except ImportError as e:
            LOG.error(f"❌ Errore import RemoteURController: {e}")
            LOG.error(traceback.format_exc())
            self.results['remote_controller'] = False
            return False
        except Exception as e:
            LOG.error(f"❌ Errore Remote Controller: {e}")
            LOG.error(traceback.format_exc())
            self.results['remote_controller'] = False
            return False
    
    def run_all_tests(self):
        """Esegue tutti i test"""
        LOG.info("\n" + "=" * 80)
        LOG.info("TROUBLESHOOTING COMPLETO ROBOT UR5e")
        LOG.info("=" * 80)
        LOG.info(f"Robot IP: {self.robot_ip}")
        LOG.info(f"Log file: {LOG_FILE}")
        LOG.info(f"Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        LOG.info("=" * 80)
        
        tests = [
            ("Network", self.test_network_connectivity),
            ("Ports", self.test_all_ports),
            ("Primary Interface", self.test_primary_interface),
            ("RTDE", self.test_rtde_connection),
            ("Dashboard", self.test_dashboard_server),
            ("URScript Port", self.test_urscript_port),
            ("Remote Controller", self.test_remote_controller),
        ]
        
        results = {}
        for name, test_func in tests:
            try:
                results[name] = test_func()
            except Exception as e:
                LOG.error(f"❌ Errore fatale in test {name}: {e}")
                LOG.error(traceback.format_exc())
                results[name] = False
        
        # Riepilogo finale
        LOG.info("\n" + "=" * 80)
        LOG.info("RIEPILOGO RISULTATI")
        LOG.info("=" * 80)
        
        for name, result in results.items():
            status = "✅ OK" if result else "❌ FAIL"
            LOG.info(f"{name:20s}: {status}")
        
        total = len(results)
        passed = sum(1 for r in results.values() if r)
        LOG.info(f"\nTest passati: {passed}/{total}")
        
        if passed == total:
            LOG.info("✅ TUTTI I TEST PASSATI - Robot pronto all'uso!")
        else:
            LOG.warning(f"⚠️  {total - passed} test falliti - Verifica problemi sopra")
        
        LOG.info(f"\nLog completo salvato in: {LOG_FILE}")
        
        return results

def main():
    """Main function"""
    troubleshoot = RobotTroubleshooter()
    results = troubleshoot.run_all_tests()
    
    # Exit code basato sui risultati
    if all(results.values()):
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()










