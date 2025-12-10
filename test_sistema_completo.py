#!/usr/bin/env python3
"""
Script di Verifica Completo Sistema AI Accelerator
Universal Robots + ROS2 + MuJoCo + Orbbec Vision

Esegue verifiche complete di tutti i componenti installati e funzionanti.
"""

import subprocess
import sys
import os
import time
import socket
from typing import Dict, List, Tuple, Optional
from datetime import datetime

# Colori per output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_header(text: str):
    """Stampa header formattato"""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text.center(80)}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*80}{Colors.RESET}\n")

def print_success(text: str):
    """Stampa messaggio di successo"""
    print(f"{Colors.GREEN}✅ {text}{Colors.RESET}")

def print_error(text: str):
    """Stampa messaggio di errore"""
    print(f"{Colors.RED}❌ {text}{Colors.RESET}")

def print_warning(text: str):
    """Stampa messaggio di warning"""
    print(f"{Colors.YELLOW}⚠️  {text}{Colors.RESET}")

def print_info(text: str):
    """Stampa messaggio informativo"""
    print(f"{Colors.BLUE}ℹ️  {text}{Colors.RESET}")

def check_command(command: List[str], timeout: int = 5) -> Tuple[bool, str, str]:
    """Esegue comando e ritorna (success, stdout, stderr)"""
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Timeout"
    except Exception as e:
        return False, "", str(e)

def check_python_package(package_name: str, import_name: Optional[str] = None) -> Tuple[bool, str]:
    """Verifica se un pacchetto Python è installato"""
    if import_name is None:
        import_name = package_name
    try:
        mod = __import__(import_name)
        version = getattr(mod, '__version__', 'OK')
        return True, str(version)
    except ImportError:
        return False, ""

def check_ros2_package(package_name: str) -> Tuple[bool, List[str]]:
    """Verifica se un pacchetto ROS2 è installato"""
    success, stdout, _ = check_command(['ros2', 'pkg', 'list'], timeout=10)
    if not success:
        return False, []
    packages = [p.strip() for p in stdout.split('\n') if p.strip()]
    found = [p for p in packages if package_name.lower() in p.lower()]
    return len(found) > 0, found

def check_network_connectivity(host: str, port: int = 22, timeout: int = 3) -> bool:
    """Verifica connettività di rete"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except:
        return False

def check_file_exists(path: str) -> bool:
    """Verifica se un file esiste"""
    return os.path.exists(os.path.expanduser(path))

def check_directory_exists(path: str) -> bool:
    """Verifica se una directory esiste"""
    return os.path.isdir(os.path.expanduser(path))

# ============================================================================
# PARTE 1: SISTEMA OPERATIVO E BASE
# ============================================================================

def test_sistema_base() -> Dict:
    """Test sistema operativo e componenti base"""
    results = {
        'ubuntu_version': None,
        'python_version': None,
        'ros2_installed': False,
        'ros2_distro': None,
        'docker_installed': False,
    }
    
    print_header("1. SISTEMA OPERATIVO E BASE")
    
    # Ubuntu version
    success, stdout, _ = check_command(['lsb_release', '-rs'])
    if success:
        ubuntu_version = stdout.strip()
        results['ubuntu_version'] = ubuntu_version
        if ubuntu_version.startswith('22'):
            print_success(f"Ubuntu {ubuntu_version} (compatibile)")
        else:
            print_warning(f"Ubuntu {ubuntu_version} (richiesto 22.04)")
    else:
        print_warning("Impossibile verificare versione Ubuntu")
    
    # Python version
    python_version = sys.version_info
    results['python_version'] = f"{python_version.major}.{python_version.minor}.{python_version.micro}"
    if python_version >= (3, 10):
        print_success(f"Python {results['python_version']}")
    else:
        print_error(f"Python {results['python_version']} (richiesto 3.10+)")
    
    # ROS2
    success, stdout, _ = check_command(['ros2', '--version'], timeout=5)
    if success:
        results['ros2_installed'] = True
        print_success("ROS2 installato")
        print_info(f"  {stdout}")
        
        # ROS2 distro
        ros_distro = os.environ.get('ROS_DISTRO', '')
        if ros_distro:
            results['ros2_distro'] = ros_distro
            print_info(f"  ROS_DISTRO: {ros_distro}")
        else:
            # Prova a source e verificare
            success, stdout, _ = check_command(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash && echo $ROS_DISTRO'],
                timeout=5
            )
            if success and stdout:
                results['ros2_distro'] = stdout.strip()
                print_info(f"  ROS_DISTRO: {results['ros2_distro']}")
    else:
        print_error("ROS2 NON installato")
    
    # Docker
    success, _, _ = check_command(['docker', '--version'], timeout=3)
    if success:
        results['docker_installed'] = True
        print_success("Docker disponibile")
    else:
        print_warning("Docker non trovato (opzionale per URSim)")
    
    return results

# ============================================================================
# PARTE 2: DRIVER E INTERFACCE UR
# ============================================================================

def test_driver_ur() -> Dict:
    """Test driver UR e interfacce"""
    results = {
        'ur_ros2_driver': False,
        'ur_ros2_packages': [],
        'ur_rtde': False,
        'rtde_library': False,
        'workspace_exists': False,
        'driver_repo_exists': False,
    }
    
    print_header("2. DRIVER E INTERFACCE UR")
    
    # UR ROS2 Driver (pacchetti installati)
    found, packages = check_ros2_package('ur_robot_driver')
    if found:
        results['ur_ros2_driver'] = True
        results['ur_ros2_packages'] = packages
        print_success("UR ROS2 Driver installato")
        for pkg in packages[:5]:
            print_info(f"  - {pkg}")
    else:
        print_error("UR ROS2 Driver NON installato")
    
    # Workspace ROS2
    ros2_ws = os.path.expanduser('~/ros2_ws/src')
    if check_directory_exists(ros2_ws):
        results['workspace_exists'] = True
        print_success(f"ROS2 workspace trovato: {ros2_ws}")
        
        # Universal_Robots_ROS2_Driver
        ur_driver_dir = os.path.join(ros2_ws, 'Universal_Robots_ROS2_Driver')
        if check_directory_exists(ur_driver_dir):
            results['driver_repo_exists'] = True
            print_success("Universal_Robots_ROS2_Driver presente")
        else:
            print_error("Universal_Robots_ROS2_Driver NON presente")
    else:
        print_warning(f"ROS2 workspace NON trovato: {ros2_ws}")
    
    # ur_rtde
    found, version = check_python_package('ur_rtde')
    if found:
        results['ur_rtde'] = True
        print_success(f"ur_rtde installato ({version})")
    else:
        print_error("ur_rtde NON installato (pip install ur-rtde)")
    
    # RTDE Python Library
    found, version = check_python_package('rtde')
    if found:
        results['rtde_library'] = True
        print_success(f"RTDE Python Library installato ({version})")
    else:
        print_warning("RTDE Python Library non trovato (opzionale)")
    
    return results

# ============================================================================
# PARTE 3: MUJOCO SIMULATION
# ============================================================================

def test_mujoco() -> Dict:
    """Test MuJoCo e modelli"""
    results = {
        'mujoco_installed': False,
        'mujoco_version': None,
        'menagerie_exists': False,
        'ur5e_model': False,
        'ur10e_model': False,
    }
    
    print_header("3. MUJOCO SIMULATION")
    
    # MuJoCo
    found, version = check_python_package('mujoco')
    if found:
        results['mujoco_installed'] = True
        results['mujoco_version'] = version
        print_success(f"MuJoCo installato ({version})")
    else:
        print_error("MuJoCo NON installato (pip install mujoco)")
    
    # MuJoCo Menagerie
    mujoco_menagerie = os.path.expanduser('~/mujoco_menagerie')
    if check_directory_exists(mujoco_menagerie):
        results['menagerie_exists'] = True
        print_success(f"MuJoCo Menagerie trovato: {mujoco_menagerie}")
        
        # Modelli UR
        ur5e_model = os.path.join(mujoco_menagerie, 'universal_robots_ur5e')
        ur10e_model = os.path.join(mujoco_menagerie, 'universal_robots_ur10e')
        
        if check_directory_exists(ur5e_model):
            results['ur5e_model'] = True
            print_success("Modello UR5e presente")
        else:
            print_warning("Modello UR5e NON presente")
        
        if check_directory_exists(ur10e_model):
            results['ur10e_model'] = True
            print_success("Modello UR10e presente")
        else:
            print_warning("Modello UR10e NON presente")
    else:
        print_error(f"MuJoCo Menagerie NON trovato (git clone google-deepmind/mujoco_menagerie)")
    
    return results

# ============================================================================
# PARTE 4: ORBBEC CAMERA SDK
# ============================================================================

def test_orbbec_camera() -> Dict:
    """Test Orbbec Camera SDK"""
    results = {
        'orbbec_ros2': False,
        'orbbec_packages': [],
        'orbbec_repo_exists': False,
        'orbbec_sdk_python': False,
    }
    
    print_header("4. ORBBEC CAMERA SDK")
    
    # OrbbecSDK ROS2
    found, packages = check_ros2_package('orbbec')
    if found:
        results['orbbec_ros2'] = True
        results['orbbec_packages'] = packages
        print_success("OrbbecSDK ROS2 installato")
        for pkg in packages[:5]:
            print_info(f"  - {pkg}")
    else:
        print_warning("OrbbecSDK ROS2 non trovato nei pacchetti ROS2")
    
    # Repository OrbbecSDK_ROS2
    ros2_ws = os.path.expanduser('~/ros2_ws/src')
    orbbec_dir = os.path.join(ros2_ws, 'OrbbecSDK_ROS2')
    if check_directory_exists(orbbec_dir):
        results['orbbec_repo_exists'] = True
        print_success("OrbbecSDK_ROS2 presente in workspace")
    else:
        print_warning("OrbbecSDK_ROS2 NON presente in workspace")
    
    # OrbbecSDK Python
    found, version = check_python_package('orbbecsdk')
    if found:
        results['orbbec_sdk_python'] = True
        print_success(f"OrbbecSDK Python installato ({version})")
    else:
        print_warning("OrbbecSDK Python non trovato (opzionale)")
    
    return results

# ============================================================================
# PARTE 5: OBJECT DETECTION E AI
# ============================================================================

def test_ai_components() -> Dict:
    """Test componenti AI e computer vision"""
    results = {
        'yolov8': False,
        'opencv': False,
        'open3d': False,
        'isaac_ros': False,
    }
    
    print_header("5. OBJECT DETECTION E AI")
    
    # YOLOv8
    found, version = check_python_package('ultralytics')
    if found:
        results['yolov8'] = True
        print_success(f"YOLOv8 (ultralytics) installato ({version})")
    else:
        print_error("YOLOv8 NON installato (pip install ultralytics)")
    
    # OpenCV
    found, version = check_python_package('cv2', 'cv2')
    if found:
        results['opencv'] = True
        print_success(f"OpenCV installato ({version})")
    else:
        print_error("OpenCV NON installato (pip install opencv-python)")
    
    # Open3D
    found, version = check_python_package('open3d')
    if found:
        results['open3d'] = True
        print_success(f"Open3D installato ({version})")
    else:
        print_warning("Open3D NON installato (pip install open3d)")
    
    # Isaac ROS
    found, packages = check_ros2_package('isaac')
    if found:
        results['isaac_ros'] = True
        print_success("Isaac ROS installato")
        for pkg in packages[:3]:
            print_info(f"  - {pkg}")
    else:
        print_warning("Isaac ROS non trovato (integrato in AI Accelerator 1.1)")
    
    return results

# ============================================================================
# PARTE 6: MOTION PLANNING
# ============================================================================

def test_motion_planning() -> Dict:
    """Test MoveIt2 e motion planning"""
    results = {
        'moveit2': False,
        'moveit_packages': [],
        'ros2_control': False,
    }
    
    print_header("6. MOTION PLANNING")
    
    # MoveIt2
    found, packages = check_ros2_package('moveit')
    if found:
        results['moveit2'] = True
        results['moveit_packages'] = packages
        print_success(f"MoveIt2 installato ({len(packages)} pacchetti)")
        for pkg in packages[:5]:
            print_info(f"  - {pkg}")
    else:
        print_error("MoveIt2 NON installato (sudo apt install ros-humble-moveit)")
    
    # ros2_control
    found, packages = check_ros2_package('ros2_control')
    if found:
        results['ros2_control'] = True
        print_success("ros2_control installato")
    else:
        print_warning("ros2_control non trovato (incluso nel driver UR)")
    
    return results

# ============================================================================
# PARTE 7: COMPONENTI CUSTOM
# ============================================================================

def test_componenti_custom() -> Dict:
    """Test componenti custom del progetto"""
    results = {
        'web_interface': False,
        'ros2_bridge': False,
        'remote_controller': False,
        'project_root': False,
    }
    
    print_header("7. COMPONENTI CUSTOM")
    
    # Project root
    project_root = os.path.expanduser('~/MekoAiAccelerator')
    if check_directory_exists(project_root):
        results['project_root'] = True
        print_success(f"Project root trovato: {project_root}")
    else:
        print_warning(f"Project root NON trovato: {project_root}")
    
    # Web Interface
    web_interface = os.path.join(project_root, 'remote_ur_control', 'web_interface.py')
    if check_file_exists(web_interface):
        results['web_interface'] = True
        print_success("Web Interface presente")
    else:
        print_error("Web Interface NON presente")
    
    # ROS2 Bridge
    ros2_bridge = os.path.join(project_root, 'ros2_bridge_fixed.py')
    if check_file_exists(ros2_bridge):
        results['ros2_bridge'] = True
        print_success("ROS2 Bridge presente")
    else:
        print_error("ROS2 Bridge NON presente")
    
    # Remote UR Controller
    remote_controller = os.path.join(project_root, 'remote_ur_control', 'remote_ur_controller.py')
    if check_file_exists(remote_controller):
        results['remote_controller'] = True
        print_success("Remote UR Controller presente")
    else:
        print_error("Remote UR Controller NON presente")
    
    return results

# ============================================================================
# PARTE 8: CONNETTIVITÀ DI RETE
# ============================================================================

def test_connettivita() -> Dict:
    """Test connettività di rete"""
    results = {
        'robot_reachable': False,
        'ai_accelerator_reachable': False,
        'robot_ip': '192.168.10.194',
        'ai_accelerator_ip': '192.168.10.191',
    }
    
    print_header("8. CONNETTIVITÀ DI RETE")
    
    # Robot
    robot_ip = results['robot_ip']
    if check_network_connectivity(robot_ip, port=22):
        results['robot_reachable'] = True
        print_success(f"Robot raggiungibile: {robot_ip}")
    else:
        print_error(f"Robot NON raggiungibile: {robot_ip}")
    
    # AI Accelerator (siamo già qui, ma verifichiamo comunque)
    ai_ip = results['ai_accelerator_ip']
    if check_network_connectivity(ai_ip, port=22):
        results['ai_accelerator_reachable'] = True
        print_success(f"AI Accelerator raggiungibile: {ai_ip}")
    else:
        print_warning(f"AI Accelerator NON raggiungibile: {ai_ip} (siamo già qui?)")
    
    return results

# ============================================================================
# PARTE 9: ROS2 TOPICS E NODI
# ============================================================================

def test_ros2_runtime() -> Dict:
    """Test runtime ROS2 (topics, nodi)"""
    results = {
        'ros2_available': False,
        'topics_available': False,
        'nodes_available': False,
        'ur_nodes': [],
        'ur_topics': [],
    }
    
    print_header("9. ROS2 RUNTIME (Topics e Nodi)")
    
    # Verifica ROS2 disponibile
    success, _, _ = check_command(['ros2', '--version'], timeout=5)
    if not success:
        print_error("ROS2 non disponibile")
        return results
    
    results['ros2_available'] = True
    
    # Topics
    success, stdout, _ = check_command(['ros2', 'topic', 'list'], timeout=5)
    if success and stdout:
        results['topics_available'] = True
        topics = [t.strip() for t in stdout.split('\n') if t.strip()]
        ur_topics = [t for t in topics if 'ur' in t.lower() or 'joint' in t.lower() or 'servo' in t.lower()]
        results['ur_topics'] = ur_topics
        print_success(f"Topics disponibili: {len(topics)}")
        if ur_topics:
            print_info("Topics UR rilevati:")
            for topic in ur_topics[:10]:
                print_info(f"  - {topic}")
        else:
            print_warning("Nessun topic UR rilevato (driver non avviato?)")
    else:
        print_warning("Nessun topic disponibile (ROS2 daemon non avviato?)")
    
    # Nodes
    success, stdout, _ = check_command(['ros2', 'node', 'list'], timeout=5)
    if success and stdout:
        results['nodes_available'] = True
        nodes = [n.strip() for n in stdout.split('\n') if n.strip()]
        ur_nodes = [n for n in nodes if 'ur' in n.lower()]
        results['ur_nodes'] = ur_nodes
        print_success(f"Nodi disponibili: {len(nodes)}")
        if ur_nodes:
            print_info("Nodi UR rilevati:")
            for node in ur_nodes[:10]:
                print_info(f"  - {node}")
        else:
            print_warning("Nessun nodo UR rilevato (driver non avviato?)")
    else:
        print_warning("Nessun nodo disponibile (ROS2 daemon non avviato?)")
    
    return results

# ============================================================================
# RIEPILOGO FINALE
# ============================================================================

def print_riepilogo(all_results: Dict):
    """Stampa riepilogo completo"""
    print_header("RIEPILOGO COMPLETO")
    
    # Conta componenti
    total_checks = 0
    passed_checks = 0
    failed_checks = 0
    warning_checks = 0
    
    def count_results(section: Dict, required: List[str] = None):
        nonlocal total_checks, passed_checks, failed_checks, warning_checks
        for key, value in section.items():
            if isinstance(value, bool):
                total_checks += 1
                if value:
                    passed_checks += 1
                elif required and key in required:
                    failed_checks += 1
                else:
                    warning_checks += 1
            elif isinstance(value, list) and len(value) > 0:
                total_checks += 1
                passed_checks += 1
    
    # Componenti richiesti
    required_base = ['ros2_installed', 'python_version']
    required_driver = ['ur_ros2_driver', 'ur_rtde']
    required_ai = ['opencv']
    
    count_results(all_results.get('sistema_base', {}), required_base)
    count_results(all_results.get('driver_ur', {}), required_driver)
    count_results(all_results.get('mujoco', {}))
    count_results(all_results.get('orbbec', {}))
    count_results(all_results.get('ai', {}), required_ai)
    count_results(all_results.get('motion_planning', {}))
    count_results(all_results.get('componenti_custom', {}))
    count_results(all_results.get('connettivita', {}))
    count_results(all_results.get('ros2_runtime', {}))
    
    print(f"\n{Colors.BOLD}Statistiche:{Colors.RESET}")
    print(f"  {Colors.GREEN}✅ Componenti OK: {passed_checks}{Colors.RESET}")
    print(f"  {Colors.RED}❌ Componenti mancanti (richiesti): {failed_checks}{Colors.RESET}")
    print(f"  {Colors.YELLOW}⚠️  Componenti opzionali/mancanti: {warning_checks}{Colors.RESET}")
    print(f"  {Colors.BLUE}📊 Totale verifiche: {total_checks}{Colors.RESET}")
    
    # Stato generale
    print(f"\n{Colors.BOLD}Stato Generale:{Colors.RESET}")
    if failed_checks == 0:
        print_success("Sistema pronto per l'uso!")
    elif failed_checks <= 2:
        print_warning("Sistema quasi pronto - alcuni componenti mancanti")
    else:
        print_error("Sistema incompleto - installare componenti mancanti")
    
    # Prossimi passi
    print(f"\n{Colors.BOLD}Prossimi Passi:{Colors.RESET}")
    if not all_results.get('sistema_base', {}).get('ros2_installed'):
        print_info("1. Installare ROS2 Humble")
    if not all_results.get('driver_ur', {}).get('ur_ros2_driver'):
        print_info("2. Installare UR ROS2 Driver")
    if not all_results.get('driver_ur', {}).get('ur_rtde'):
        print_info("3. Installare ur_rtde: pip install ur-rtde")
    if not all_results.get('ai', {}).get('yolov8'):
        print_info("4. Installare YOLOv8: pip install ultralytics")
    if not all_results.get('mujoco', {}).get('mujoco_installed'):
        print_info("5. Installare MuJoCo: pip install mujoco")
    
    print()

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Esegue tutti i test"""
    print_header("VERIFICA COMPLETA SISTEMA AI ACCELERATOR")
    print(f"Data/Ora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Hostname: {os.uname().nodename if hasattr(os, 'uname') else 'N/A'}")
    
    all_results = {}
    
    # Esegui tutti i test
    all_results['sistema_base'] = test_sistema_base()
    all_results['driver_ur'] = test_driver_ur()
    all_results['mujoco'] = test_mujoco()
    all_results['orbbec'] = test_orbbec_camera()
    all_results['ai'] = test_ai_components()
    all_results['motion_planning'] = test_motion_planning()
    all_results['componenti_custom'] = test_componenti_custom()
    all_results['connettivita'] = test_connettivita()
    all_results['ros2_runtime'] = test_ros2_runtime()
    
    # Riepilogo
    print_riepilogo(all_results)
    
    # Salva risultati in file
    import json
    results_file = os.path.expanduser('~/test_sistema_results.json')
    with open(results_file, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)
    print_info(f"Risultati salvati in: {results_file}")

if __name__ == '__main__':
    main()





