#!/usr/bin/env python3
"""
Script completo per verificare lo stato del sistema dopo aver disabilitato EtherNet/IP
e guidare l'utente nei prossimi passi.
"""

import os
import sys
import socket
import subprocess
import time
from typing import Dict, List, Tuple

ROBOT_IP = os.environ.get("UR_ROBOT_IP", "192.168.10.194")
DASHBOARD_PORT = 29999
RTDE_PORT = 30004
ROS2_DRIVER_PORT = 50002

def print_header(text: str):
    """Stampa un header formattato"""
    print("\n" + "=" * 80)
    print(f"  {text}")
    print("=" * 80 + "\n")

def print_status(check: str, status: bool, message: str = ""):
    """Stampa lo stato di un check"""
    icon = "✅" if status else "❌"
    print(f"{icon} {check}")
    if message:
        print(f"   {message}")

def print_warning(check: str, message: str):
    """Stampa un avviso"""
    print(f"⚠️  {check}")
    print(f"   {message}")

def test_robot_ping() -> bool:
    """Test ping al robot"""
    try:
        result = subprocess.run(
            ['ping', '-c', '2', '-W', '2', ROBOT_IP],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except:
        return False

def test_port_open(ip: str, port: int, timeout: float = 2.0) -> bool:
    """Test se una porta è aperta"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except:
        return False

def get_dashboard_status() -> Dict[str, str]:
    """Legge lo stato Dashboard dal robot"""
    status = {}
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect((ROBOT_IP, DASHBOARD_PORT))
        
        # Leggi welcome message
        sock.recv(1024)
        
        commands = {
            "robotmode": "robotmode",
            "safetymode": "safetymode",
            "programState": "programState",
            "remote_control": "is in remote control",
        }
        
        for key, cmd in commands.items():
            try:
                sock.sendall((cmd + "\n").encode('utf-8'))
                time.sleep(0.15)
                response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
                status[key] = response
            except:
                status[key] = "unknown"
        
        sock.close()
    except Exception as e:
        status["error"] = str(e)
    
    return status

def check_ros2_driver_active() -> Tuple[bool, str]:
    """Verifica se il driver ROS2 è attivo"""
    # Controlla porta 50002
    port_listening = test_port_open("0.0.0.0", ROS2_DRIVER_PORT) or test_port_open("127.0.0.1", ROS2_DRIVER_PORT)
    
    # Controlla processi
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'ur_robot_driver|ur_control.launch'],
            capture_output=True,
            text=True
        )
        process_running = result.returncode == 0
    except:
        process_running = False
    
    # Controlla topic ROS2
    topic_active = False
    try:
        result = subprocess.run(
            ['timeout', '2', 'ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0 and 'joint_states' in result.stdout:
            topic_active = True
    except:
        pass
    
    if port_listening or process_running or topic_active:
        details = []
        if port_listening:
            details.append("porta 50002 in ascolto")
        if process_running:
            details.append("processo attivo")
        if topic_active:
            details.append("topic ROS2 attivi")
        return True, ", ".join(details)
    
    return False, ""

def check_rtde_available() -> Tuple[bool, str]:
    """Verifica se RTDE è disponibile (senza overflow)"""
    port_open = test_port_open(ROBOT_IP, RTDE_PORT)
    
    if not port_open:
        return False, "Porta RTDE (30004) non raggiungibile"
    
    # Prova connessione RTDE leggera
    try:
        import rtde.rtde as rtde_lib
        rtde_client = rtde_lib.RTDE(ROBOT_IP, RTDE_PORT)
        rtde_client.connect()
        
        # Setup minimale per test
        rtde_client.send_output_setup(['actual_q'], frequency=10)
        rtde_client.send_start()
        
        # Prova a ricevere un dato
        state = rtde_client.receive()
        
        rtde_client.send_pause()
        rtde_client.disconnect()
        
        if state:
            return True, "RTDE disponibile e funzionante"
        else:
            return False, "RTDE connesso ma nessun dato ricevuto"
    except ImportError:
        return False, "Libreria ur_rtde non installata"
    except Exception as e:
        error_msg = str(e)
        if "overflow" in error_msg.lower():
            return False, f"RTDE overflow rilevato: {error_msg}"
        return False, f"Errore RTDE: {error_msg}"

def main():
    print_header("VERIFICA STATO DOPO DISABILITAZIONE ETHERNET/IP")
    
    print(f"Robot IP: {ROBOT_IP}\n")
    
    # Checklist risultati
    checks = {
        "robot_reachable": False,
        "dashboard_available": False,
        "robot_running": False,
        "program_playing": False,
        "remote_control": False,
        "rtde_available": False,
        "ros2_driver_active": False,
        "port_50002_listening": False,
    }
    
    messages = {}
    
    # 1. Test connessione robot
    print("1. Test connessione robot...")
    if test_robot_ping():
        print_status("Robot raggiungibile", True)
        checks["robot_reachable"] = True
    else:
        print_status("Robot raggiungibile", False, "Verifica connettività di rete")
        print("\n❌ ERRORE CRITICO: Robot non raggiungibile")
        print("   Verifica:")
        print("   - Robot acceso")
        print("   - IP corretto")
        print("   - Connettività di rete")
        return 1
    
    # 2. Verifica Dashboard
    print("\n2. Verifica Dashboard Server (porta 29999)...")
    if test_port_open(ROBOT_IP, DASHBOARD_PORT):
        print_status("Dashboard Server raggiungibile", True)
        checks["dashboard_available"] = True
        
        # Leggi stato Dashboard
        print("\n   Lettura stato robot...")
        dashboard_status = get_dashboard_status()
        
        if "error" in dashboard_status:
            print_warning("Errore lettura Dashboard", dashboard_status["error"])
        else:
            robot_mode = dashboard_status.get("robotmode", "unknown")
            safety_mode = dashboard_status.get("safetymode", "unknown")
            program_state = dashboard_status.get("programState", "unknown")
            remote_control = dashboard_status.get("remote_control", "unknown")
            
            print(f"\n   Robot Mode: {robot_mode}")
            print(f"   Safety Mode: {safety_mode}")
            print(f"   Program State: {program_state}")
            print(f"   Remote Control: {remote_control}")
            
            # Verifica condizioni
            if "RUNNING" in robot_mode.upper():
                checks["robot_running"] = True
                print_status("   Robot in RUNNING", True)
            else:
                print_warning("   Robot NON in RUNNING", f"Stato attuale: {robot_mode}")
            
            if "PLAYING" in program_state.upper():
                checks["program_playing"] = True
                print_status("   Programma in PLAYING", True)
            else:
                print_warning("   Programma NON in PLAYING", f"Stato attuale: {program_state}")
            
            if "true" in remote_control.lower():
                checks["remote_control"] = True
                print_status("   Remote Control abilitato", True)
            else:
                print_warning("   Remote Control NON abilitato", f"Stato attuale: {remote_control}")
    else:
        print_status("Dashboard Server raggiungibile", False, "Porta 29999 non raggiungibile")
    
    # 3. Verifica RTDE
    print("\n3. Verifica RTDE (porta 30004)...")
    rtde_ok, rtde_msg = check_rtde_available()
    checks["rtde_available"] = rtde_ok
    if rtde_ok:
        print_status("RTDE disponibile", True, rtde_msg)
    else:
        print_warning("RTDE non disponibile", rtde_msg)
        if "overflow" in rtde_msg.lower():
            print("\n   ⚠️  ATTENZIONE: RTDE overflow rilevato!")
            print("   Questo potrebbe indicare che:")
            print("   - EtherNet/IP è ancora attivo (verifica sul Teach Pendant)")
            print("   - Altri processi stanno usando RTDE")
            print("   - Driver ROS2 già in esecuzione")
    
    # 4. Verifica driver ROS2
    print("\n4. Verifica driver ROS2...")
    ros2_active, ros2_details = check_ros2_driver_active()
    checks["ros2_driver_active"] = ros2_active
    
    if ros2_active:
        print_status("Driver ROS2 attivo", True, ros2_details)
        checks["port_50002_listening"] = test_port_open("0.0.0.0", ROS2_DRIVER_PORT) or test_port_open("127.0.0.1", ROS2_DRIVER_PORT)
        
        if checks["port_50002_listening"]:
            print_status("   Porta 50002 in ascolto", True)
        else:
            print_warning("   Porta 50002", "Non in ascolto (ma driver attivo)")
    else:
        print_status("Driver ROS2 attivo", False, "Driver non in esecuzione")
        print("\n   ℹ️  Il driver ROS2 non è ancora avviato")
        print("   Questo è normale se stai appena iniziando")
    
    # 5. Riepilogo e prossimi passi
    print_header("RIEPILOGO STATO")
    
    all_ok = all([
        checks["robot_reachable"],
        checks["dashboard_available"],
        checks["robot_running"],
        checks["program_playing"],
        checks["remote_control"],
        checks["rtde_available"],
    ])
    
    print("\nChecklist:")
    print(f"  {'✅' if checks['robot_reachable'] else '❌'} Robot raggiungibile")
    print(f"  {'✅' if checks['dashboard_available'] else '❌'} Dashboard Server disponibile")
    print(f"  {'✅' if checks['robot_running'] else '❌'} Robot in RUNNING")
    print(f"  {'✅' if checks['program_playing'] else '❌'} Programma in PLAYING")
    print(f"  {'✅' if checks['remote_control'] else '❌'} Remote Control abilitato")
    print(f"  {'✅' if checks['rtde_available'] else '❌'} RTDE disponibile (senza overflow)")
    print(f"  {'✅' if checks['ros2_driver_active'] else '⏸️ '} Driver ROS2 attivo")
    print(f"  {'✅' if checks['port_50002_listening'] else '⏸️ '} Porta 50002 in ascolto")
    
    print_header("PROSSIMI PASSI")
    
    if not all_ok:
        print("\n⚠️  Alcuni check non sono passati. Risolvi prima di procedere:\n")
        
        if not checks["robot_running"]:
            print("1. Sul Teach Pendant:")
            print("   - Verifica che il robot sia acceso")
            print("   - Premi il pulsante di avvio (se necessario)")
            print("   - Attendi che entri in modalità RUNNING\n")
        
        if not checks["program_playing"]:
            print("2. Sul Teach Pendant:")
            print("   - Crea/modifica programma con nodo External Control")
            print("   - Configura:")
            print("     * Host IP: 192.168.10.191")
            print("     * Port: 50002")
            print("   - NON avviare ancora (lascia in STOP)\n")
        
        if not checks["remote_control"]:
            print("3. Sul Teach Pendant:")
            print("   - Settings → System → Remote Control")
            print("   - Abilita Remote Control\n")
        
        if not checks["rtde_available"]:
            print("4. Verifica EtherNet/IP:")
            print("   - Sul Teach Pendant: Installation → Fieldbus")
            print("   - Assicurati che EtherNet/IP sia DISABILITATO")
            print("   - Assicurati che PROFINET sia DISABILITATO")
            print("   - Riavvia il robot dopo aver disabilitato\n")
    
    if all_ok and not checks["ros2_driver_active"]:
        print("\n✅ Tutti i check base sono OK!")
        print("\n📋 Ora puoi avviare il driver ROS2:\n")
        print("   ssh lab@192.168.10.191")
        print("   cd ~/ros2_ws")
        print("   source /opt/ros/humble/setup.bash")
        print("   source install/setup.bash")
        print("   ros2 launch ur_robot_driver ur_control.launch.py \\")
        print("       ur_type:=ur5e \\")
        print("       robot_ip:=192.168.10.194 \\")
        print("       launch_rviz:=false\n")
        print("   Oppure usa lo script:")
        print("   ./metodo_guida_pratica/VERIFICA_E_AVVIA_DOPO_ETHERNET_IP.sh\n")
    
    elif all_ok and checks["ros2_driver_active"]:
        print("\n✅ TUTTO OK! Il sistema è pronto.\n")
        print("📋 Verifica nel log del driver ROS2:")
        print("   - Dovresti vedere: 'System successfully started!'")
        print("   - NON dovresti vedere: 'Pipeline producer overflowed!'")
        print("   - NON dovresti vedere: 'Segmentation fault'\n")
        print("📋 Sul Teach Pendant:")
        print("   - Verifica che External Control sia configurato correttamente")
        print("   - Premi PLAY per connettere il robot al driver\n")
    
    return 0 if all_ok else 1

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrotto dall'utente")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ ERRORE: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)







