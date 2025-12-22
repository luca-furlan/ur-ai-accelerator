#!/usr/bin/env python3
"""
Test completo sistema Vision + Robot

Verifica:
1. Camera Orbecc disponibilità
2. YOLOv8 funzionamento
3. ROS2 topics
4. Vision nodes online
5. Robot connection
"""

import subprocess
import time
import sys
import json

def check_command(cmd):
    """Esegue comando e verifica successo"""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0, result.stdout
    except:
        return False, ""


def print_header(text):
    print("\n" + "=" * 60)
    print(text)
    print("=" * 60)


def print_success(text):
    print(f"✅ {text}")


def print_error(text):
    print(f"❌ {text}")


def print_warning(text):
    print(f"⚠️  {text}")


def print_info(text):
    print(f"ℹ️  {text}")


def main():
    print_header("TEST SISTEMA VISION + ROBOT")
    
    results = {
        'camera': False,
        'yolo': False,
        'ros2': False,
        'nodes': False,
        'robot': False
    }
    
    # ========================================
    # 1. Test Camera Orbecc
    # ========================================
    print_header("1. CAMERA ORBECC")
    
    # Verifica topics camera
    success, output = check_command("ros2 topic list | grep camera")
    if success and output:
        camera_topics = output.strip().split('\n')
        results['camera'] = True
        print_success(f"Camera topics disponibili: {len(camera_topics)}")
        for topic in camera_topics[:5]:
            print(f"   - {topic}")
    else:
        print_error("Camera topics non trovati")
        print_info("Avvia camera: ros2 launch orbecc_camera gemini_330_series.launch.py")
    
    # Test camera data
    if results['camera']:
        print_info("Test camera data...")
        success, _ = check_command("timeout 2 ros2 topic echo /camera/color/image_raw --once")
        if success:
            print_success("Camera pubblica dati RGB")
        else:
            print_warning("Camera non pubblica dati (timeout)")
    
    # ========================================
    # 2. Test YOLOv8
    # ========================================
    print_header("2. YOLO DETECTION")
    
    # Import YOLOv8
    try:
        from ultralytics import YOLO
        import ultralytics
        version = ultralytics.__version__
        results['yolo'] = True
        print_success(f"YOLOv8 installato (v{version})")
        
        # Test model load
        try:
            model = YOLO('yolov8n.pt')
            print_success("Modello YOLOv8n caricato")
        except Exception as e:
            print_warning(f"Errore caricamento modello: {e}")
    
    except ImportError:
        print_error("YOLOv8 non installato")
        print_info("Installa: pip install ultralytics")
    
    # ========================================
    # 3. Test ROS2
    # ========================================
    print_header("3. ROS2 ENVIRONMENT")
    
    # Verifica ROS2
    success, output = check_command("ros2 --version")
    if success:
        results['ros2'] = True
        print_success(f"ROS2 disponibile: {output.strip()}")
    else:
        print_error("ROS2 non trovato")
        print_info("Source: source /opt/ros/humble/setup.bash")
    
    # Verifica Python packages ROS2
    if results['ros2']:
        try:
            import rclpy
            from cv_bridge import CvBridge
            print_success("Python ROS2 packages OK (rclpy, cv_bridge)")
        except ImportError as e:
            print_error(f"Python ROS2 packages mancanti: {e}")
    
    # ========================================
    # 4. Test Vision Nodes
    # ========================================
    print_header("4. VISION NODES")
    
    # Lista nodi ROS2
    success, output = check_command("ros2 node list")
    if success and output:
        nodes = output.strip().split('\n')
        print_info(f"Nodi ROS2 attivi: {len(nodes)}")
        
        # Cerca vision nodes
        vision_nodes = [n for n in nodes if 'vision' in n.lower()]
        if vision_nodes:
            results['nodes'] = True
            print_success(f"Vision nodes trovati: {len(vision_nodes)}")
            for node in vision_nodes:
                print(f"   - {node}")
        else:
            print_warning("Vision nodes non attivi")
            print_info("Avvia: ./avvia_vision_robot_system.sh")
    
    # Verifica vision topics
    success, output = check_command("ros2 topic list | grep vision")
    if success and output:
        vision_topics = output.strip().split('\n')
        print_info(f"Vision topics: {len(vision_topics)}")
        for topic in vision_topics[:5]:
            print(f"   - {topic}")
    
    # ========================================
    # 5. Test Robot Connection
    # ========================================
    print_header("5. ROBOT CONNECTION")
    
    # Test ping robot
    robot_ip = "192.168.10.194"
    success, _ = check_command(f"ping -c 1 -W 1 {robot_ip}")
    if success:
        results['robot'] = True
        print_success(f"Robot raggiungibile @ {robot_ip}")
        
        # Test RTDE
        try:
            import rtde_control
            rtde_c = rtde_control.RTDEControlInterface(robot_ip)
            print_success("RTDE connection OK")
            rtde_c.disconnect()
        except Exception as e:
            print_warning(f"RTDE connection failed: {e}")
    
    else:
        print_error(f"Robot non raggiungibile @ {robot_ip}")
        print_info("Verifica network e IP robot")
    
    # ========================================
    # 6. Test Vision Topics Data
    # ========================================
    print_header("6. VISION SYSTEM DATA")
    
    if results['nodes']:
        # Test detections topic
        print_info("Test detections topic...")
        success, output = check_command("timeout 2 ros2 topic echo /vision/detections_3d --once")
        if success and output:
            try:
                # Parse JSON
                data = json.loads(output)
                detections = data.get('detections', [])
                print_success(f"Detections topic OK - {len(detections)} objects")
                if detections:
                    for det in detections[:3]:
                        print(f"   - {det['class']} (conf: {det['confidence']:.2f})")
            except:
                print_warning("Detections topic OK ma dati non parsabili")
        else:
            print_warning("Nessuna detection ricevuta (timeout)")
    
    # ========================================
    # RIEPILOGO
    # ========================================
    print_header("RIEPILOGO TEST")
    
    total = len(results)
    passed = sum(results.values())
    
    print(f"\nRisultati: {passed}/{total} componenti OK\n")
    
    print(f"{'Componente':<20} {'Status'}")
    print("-" * 40)
    print(f"{'Camera Orbecc':<20} {'✅ OK' if results['camera'] else '❌ FAIL'}")
    print(f"{'YOLOv8':<20} {'✅ OK' if results['yolo'] else '❌ FAIL'}")
    print(f"{'ROS2':<20} {'✅ OK' if results['ros2'] else '❌ FAIL'}")
    print(f"{'Vision Nodes':<20} {'✅ OK' if results['nodes'] else '❌ FAIL'}")
    print(f"{'Robot Connection':<20} {'✅ OK' if results['robot'] else '❌ FAIL'}")
    
    print("\n" + "=" * 60)
    
    if passed == total:
        print("✅ SISTEMA COMPLETAMENTE FUNZIONANTE")
        print("\nPuoi procedere con:")
        print("  ./avvia_vision_robot_system.sh")
    elif passed >= 3:
        print("⚠️  SISTEMA PARZIALMENTE FUNZIONANTE")
        print("\nComponenti mancanti - segui istruzioni sopra")
    else:
        print("❌ SISTEMA NON FUNZIONANTE")
        print("\nRisolvi problemi critici prima di procedere")
    
    print("=" * 60)
    
    return passed == total


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)




