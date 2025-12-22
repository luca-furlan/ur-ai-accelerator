#!/usr/bin/env python3
"""
Test connessione Orbbec Camera e riconoscimento con MoveIt

Questo script verifica:
1. Connessione alla camera Orbbec (via ROS2)
2. Riconoscimento oggetti con YOLOv8
3. Disponibilità MoveIt2
4. Integrazione completa
"""

import sys
import time
import subprocess
import os

def print_header(text):
    print("\n" + "=" * 70)
    print(f"  {text}")
    print("=" * 70)

def print_success(text):
    print(f"✅ {text}")

def print_error(text):
    print(f"❌ {text}")

def print_warning(text):
    print(f"⚠️  {text}")

def print_info(text):
    print(f"ℹ️  {text}")

def check_ros2_available():
    """Verifica se ROS2 è disponibile"""
    try:
        result = subprocess.run(
            ['ros2', '--version'],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0:
            return True, result.stdout.strip()
        return False, None
    except:
        return False, None

def check_orbbec_topics():
    """Verifica topics camera Orbbec"""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            topics = [t.strip() for t in result.stdout.split('\n') if t.strip()]
            orbbec_topics = [t for t in topics if 'camera' in t.lower() or 'orbbec' in t.lower()]
            return orbbec_topics
        return []
    except:
        return []

def check_orbbec_package():
    """Verifica se pacchetto Orbbec è installato"""
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            packages = result.stdout.lower()
            return 'orbbec' in packages
        return False
    except:
        return False

def test_camera_data():
    """Test se la camera pubblica dati"""
    try:
        result = subprocess.run(
            ['timeout', '3', 'ros2', 'topic', 'echo', '/camera/color/image_raw', '--once'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0 and len(result.stdout) > 100
    except:
        return False

def test_yolo():
    """Test YOLOv8"""
    try:
        from ultralytics import YOLO
        import ultralytics
        version = ultralytics.__version__
        
        # Prova a caricare modello
        model = YOLO('yolov8n.pt')
        return True, version
    except ImportError:
        return False, None
    except Exception as e:
        return False, str(e)

def test_moveit():
    """Test MoveIt2 disponibilità"""
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            packages = result.stdout.lower()
            moveit_packages = [p for p in packages.split('\n') if 'moveit' in p.lower()]
            return len(moveit_packages) > 0, moveit_packages
        return False, []
    except:
        return False, []

def test_moveit_python():
    """Test import MoveIt Python"""
    try:
        from moveit_msgs.msg import Constraints
        from moveit_msgs.action import MoveGroup
        return True
    except ImportError:
        return False

def main():
    print_header("TEST ORBBEC CAMERA + MOVEIT RICONOSCIMENTO")
    
    results = {
        'ros2': False,
        'orbbec_package': False,
        'orbbec_topics': False,
        'orbbec_data': False,
        'yolo': False,
        'moveit': False,
        'moveit_python': False
    }
    
    # ========================================
    # 1. Test ROS2
    # ========================================
    print_header("1. VERIFICA ROS2")
    
    ros2_ok, ros2_version = check_ros2_available()
    if ros2_ok:
        results['ros2'] = True
        print_success(f"ROS2 disponibile: {ros2_version}")
    else:
        print_error("ROS2 non disponibile")
        print_info("Esegui: source /opt/ros/humble/setup.bash")
        print_info("Poi: source ~/ros2_ws/install/setup.bash")
        return False
    
    # ========================================
    # 2. Test Camera Orbbec - Pacchetto
    # ========================================
    print_header("2. VERIFICA CAMERA ORBBEC - PACCHETTO")
    
    orbbec_pkg = check_orbbec_package()
    if orbbec_pkg:
        results['orbbec_package'] = True
        print_success("Pacchetto ROS2 Orbbec installato")
    else:
        print_warning("Pacchetto ROS2 Orbbec non trovato")
        print_info("Installa: cd ~/ros2_ws/src && git clone https://github.com/orbbec/OrbbecSDK_ROS2.git")
        print_info("Poi: cd ~/ros2_ws && colcon build --packages-select orbbec_camera")
    
    # ========================================
    # 3. Test Camera Orbbec - Topics
    # ========================================
    print_header("3. VERIFICA CAMERA ORBBEC - TOPICS")
    
    orbbec_topics = check_orbbec_topics()
    if orbbec_topics:
        results['orbbec_topics'] = True
        print_success(f"Topics Orbbec disponibili: {len(orbbec_topics)}")
        print_info("Topics principali:")
        for topic in orbbec_topics[:8]:
            print(f"   - {topic}")
    else:
        print_warning("Nessun topic Orbbec rilevato")
        print_info("Avvia camera: ros2 launch orbbec_camera gemini_330_series.launch.py")
    
    # ========================================
    # 4. Test Camera Orbbec - Dati
    # ========================================
    print_header("4. VERIFICA CAMERA ORBBEC - DATI")
    
    if results['orbbec_topics']:
        print_info("Test ricezione dati dalla camera...")
        if test_camera_data():
            results['orbbec_data'] = True
            print_success("Camera pubblica dati RGB correttamente")
        else:
            print_warning("Camera non pubblica dati (timeout o camera non avviata)")
            print_info("Avvia camera in un altro terminale:")
            print_info("  ros2 launch orbbec_camera gemini_330_series.launch.py")
    else:
        print_warning("Impossibile testare dati: topics non disponibili")
    
    # ========================================
    # 5. Test YOLOv8
    # ========================================
    print_header("5. VERIFICA YOLOV8 RICONOSCIMENTO")
    
    yolo_ok, yolo_info = test_yolo()
    if yolo_ok:
        results['yolo'] = True
        print_success(f"YOLOv8 installato e funzionante (v{yolo_info})")
        print_success("Modello yolov8n.pt caricato correttamente")
    else:
        print_error("YOLOv8 non disponibile")
        if yolo_info:
            print_error(f"Errore: {yolo_info}")
        print_info("Installa: pip install ultralytics")
    
    # ========================================
    # 6. Test MoveIt2 - Pacchetti ROS2
    # ========================================
    print_header("6. VERIFICA MOVEIT2 - PACCHETTI ROS2")
    
    moveit_ok, moveit_packages = test_moveit()
    if moveit_ok:
        results['moveit'] = True
        print_success(f"MoveIt2 installato ({len(moveit_packages)} pacchetti)")
        print_info("Pacchetti principali:")
        for pkg in moveit_packages[:5]:
            print(f"   - {pkg}")
    else:
        print_warning("MoveIt2 non installato")
        print_info("Installa: sudo apt install ros-humble-moveit")
    
    # ========================================
    # 7. Test MoveIt2 - Python
    # ========================================
    print_header("7. VERIFICA MOVEIT2 - PYTHON INTERFACE")
    
    moveit_py_ok = test_moveit_python()
    if moveit_py_ok:
        results['moveit_python'] = True
        print_success("MoveIt2 Python interface disponibile")
    else:
        print_warning("MoveIt2 Python interface non disponibile")
        print_info("Installa: sudo apt install ros-humble-moveit-msgs")
    
    # ========================================
    # RIEPILOGO
    # ========================================
    print_header("RIEPILOGO TEST")
    
    total = len(results)
    passed = sum(results.values())
    
    print(f"\nRisultati: {passed}/{total} componenti OK\n")
    
    print(f"{'Componente':<30} {'Status'}")
    print("-" * 50)
    print(f"{'ROS2':<30} {'✅ OK' if results['ros2'] else '❌ FAIL'}")
    print(f"{'Orbbec Package':<30} {'✅ OK' if results['orbbec_package'] else '⚠️  MISSING'}")
    print(f"{'Orbbec Topics':<30} {'✅ OK' if results['orbbec_topics'] else '❌ FAIL'}")
    print(f"{'Orbbec Data':<30} {'✅ OK' if results['orbbec_data'] else '⚠️  NO DATA'}")
    print(f"{'YOLOv8':<30} {'✅ OK' if results['yolo'] else '❌ FAIL'}")
    print(f"{'MoveIt2 Packages':<30} {'✅ OK' if results['moveit'] else '⚠️  MISSING'}")
    print(f"{'MoveIt2 Python':<30} {'✅ OK' if results['moveit_python'] else '⚠️  MISSING'}")
    
    print("\n" + "=" * 70)
    
    # Valutazione finale
    critical_ok = results['ros2'] and results['yolo']
    camera_ok = results['orbbec_topics'] and results['orbbec_data']
    moveit_ok = results['moveit'] and results['moveit_python']
    
    if critical_ok and camera_ok and moveit_ok:
        print("✅ SISTEMA COMPLETO - Pronto per riconoscimento!")
        print("\nPuoi procedere con:")
        print("  1. Avvia camera: ros2 launch orbbec_camera gemini_330_series.launch.py")
        print("  2. Avvia vision: python3 archive/vision_yolo_detector.py")
        print("  3. Avvia MoveIt: python3 archive/moveit_vision_controller.py")
    elif critical_ok and camera_ok:
        print("⚠️  CAMERA E YOLO OK - MoveIt opzionale")
        print("\nPuoi fare riconoscimento senza MoveIt:")
        print("  1. Avvia camera: ros2 launch orbbec_camera gemini_330_series.launch.py")
        print("  2. Avvia vision: python3 archive/vision_yolo_detector.py")
    elif critical_ok:
        print("⚠️  ROS2 E YOLO OK - Avvia camera per riconoscimento")
        print("\nProssimi passi:")
        if not results['orbbec_topics']:
            print("  1. Avvia camera: ros2 launch orbbec_camera gemini_330_series.launch.py")
        print("  2. Poi riprova questo test")
    else:
        print("❌ PROBLEMI CRITICI - Risolvi prima di procedere")
        if not results['ros2']:
            print("  - Configura ROS2 environment")
        if not results['yolo']:
            print("  - Installa YOLOv8: pip install ultralytics")
    
    print("=" * 70)
    
    return critical_ok

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
