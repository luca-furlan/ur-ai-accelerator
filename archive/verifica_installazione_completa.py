#!/usr/bin/env python3
"""
Verifica completa installazione AI Accelerator secondo documentazione Universal Robots
"""

import subprocess
import sys
import os

def check_python_package(package_name, import_name=None):
    """Verifica se un pacchetto Python è installato"""
    if import_name is None:
        import_name = package_name
    try:
        __import__(import_name)
        try:
            mod = sys.modules[import_name]
            version = getattr(mod, '__version__', 'OK')
            return True, version
        except:
            return True, "OK"
    except ImportError:
        return False, None

def check_ros2_package(package_name):
    """Verifica se un pacchetto ROS2 è installato"""
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            timeout=10
        )
        packages = result.stdout.split('\n')
        found = [p for p in packages if package_name.lower() in p.lower()]
        return len(found) > 0, found
    except:
        return False, []

def check_directory(path):
    """Verifica se una directory esiste"""
    return os.path.exists(os.path.expanduser(path))

def check_file(path):
    """Verifica se un file esiste"""
    return os.path.exists(os.path.expanduser(path))

print("=" * 80)
print("VERIFICA INSTALLAZIONE AI ACCELERATOR - UNIVERSAL ROBOTS")
print("=" * 80)
print()

# Setup ROS2
os.environ.setdefault('ROS_DISTRO', 'humble')
try:
    ros2_setup = os.path.expanduser('/opt/ros/humble/setup.bash')
    if os.path.exists(ros2_setup):
        print("✅ ROS2 Humble setup trovato")
    else:
        print("❌ ROS2 Humble setup NON trovato")
except:
    pass

print("\n" + "=" * 80)
print("1. SISTEMA OPERATIVO E BASE")
print("=" * 80)

# Ubuntu version
try:
    result = subprocess.run(['lsb_release', '-rs'], capture_output=True, text=True, timeout=5)
    ubuntu_version = result.stdout.strip()
    if ubuntu_version.startswith('22'):
        print(f"✅ Ubuntu {ubuntu_version} (compatibile)")
    else:
        print(f"⚠️ Ubuntu {ubuntu_version} (richiesto 22.04)")
except:
    print("⚠️ Impossibile verificare versione Ubuntu")

# Python version
python_version = sys.version_info
if python_version >= (3, 10):
    print(f"✅ Python {python_version.major}.{python_version.minor}.{python_version.micro}")
else:
    print(f"❌ Python {python_version.major}.{python_version.minor} (richiesto 3.10+)")

# Docker
docker_ok, _ = check_python_package('docker', 'docker')
if docker_ok or subprocess.run(['which', 'docker'], capture_output=True).returncode == 0:
    print("✅ Docker disponibile")
else:
    print("⚠️ Docker non trovato (opzionale per URSim)")

print("\n" + "=" * 80)
print("2. DRIVER E INTERFACCE UR")
print("=" * 80)

# UR ROS2 Driver
ur_driver_ok, ur_driver_list = check_ros2_package('ur_robot_driver')
if ur_driver_ok:
    print(f"✅ UR ROS2 Driver installato")
    for pkg in ur_driver_list[:3]:
        print(f"   - {pkg}")
else:
    print("❌ UR ROS2 Driver NON installato")

# Verifica directory workspace
ros2_ws = os.path.expanduser('~/ros2_ws/src')
if check_directory(ros2_ws):
    print(f"✅ ROS2 workspace trovato: {ros2_ws}")
    
    # Verifica Universal_Robots_ROS2_Driver
    ur_driver_dir = os.path.join(ros2_ws, 'Universal_Robots_ROS2_Driver')
    if check_directory(ur_driver_dir):
        print(f"✅ Universal_Robots_ROS2_Driver presente")
    else:
        print(f"❌ Universal_Robots_ROS2_Driver NON presente")
else:
    print(f"❌ ROS2 workspace NON trovato")

# ur_rtde
ur_rtde_ok, ur_rtde_version = check_python_package('ur_rtde')
if ur_rtde_ok:
    print(f"✅ ur_rtde installato ({ur_rtde_version})")
else:
    print("❌ ur_rtde NON installato (pip install ur-rtde)")

# RTDE Python Library
rtde_ok, rtde_version = check_python_package('rtde')
if rtde_ok:
    print(f"✅ RTDE Python Library installato ({rtde_version})")
else:
    print("⚠️ RTDE Python Library non trovato (opzionale)")

print("\n" + "=" * 80)
print("3. MUJOCO SIMULATION")
print("=" * 80)

# MuJoCo
mujoco_ok, mujoco_version = check_python_package('mujoco')
if mujoco_ok:
    print(f"✅ MuJoCo installato ({mujoco_version})")
else:
    print("❌ MuJoCo NON installato (pip install mujoco)")

# MuJoCo Menagerie
mujoco_menagerie = os.path.expanduser('~/mujoco_menagerie')
if check_directory(mujoco_menagerie):
    print(f"✅ MuJoCo Menagerie trovato: {mujoco_menagerie}")
    
    # Verifica modelli UR
    ur5e_model = os.path.join(mujoco_menagerie, 'universal_robots_ur5e')
    ur10e_model = os.path.join(mujoco_menagerie, 'universal_robots_ur10e')
    
    if check_directory(ur5e_model):
        print(f"✅ Modello UR5e presente")
    else:
        print(f"⚠️ Modello UR5e NON presente")
        
    if check_directory(ur10e_model):
        print(f"✅ Modello UR10e presente")
    else:
        print(f"⚠️ Modello UR10e NON presente")
else:
    print(f"❌ MuJoCo Menagerie NON trovato (git clone google-deepmind/mujoco_menagerie)")

print("\n" + "=" * 80)
print("4. ORBBEC CAMERA SDK")
print("=" * 80)

# OrbbecSDK ROS2
orbbec_ok, orbbec_list = check_ros2_package('orbbec')
if orbbec_ok:
    print(f"✅ OrbbecSDK ROS2 installato")
    for pkg in orbbec_list[:3]:
        print(f"   - {pkg}")
else:
    print("⚠️ OrbbecSDK ROS2 non trovato nei pacchetti ROS2")

# Verifica directory OrbbecSDK_ROS2
orbbec_dir = os.path.join(ros2_ws, 'OrbbecSDK_ROS2')
if check_directory(orbbec_dir):
    print(f"✅ OrbbecSDK_ROS2 presente in workspace")
else:
    print(f"⚠️ OrbbecSDK_ROS2 NON presente in workspace")

# OrbbecSDK Python
orbbec_sdk_ok, _ = check_python_package('orbbecsdk')
if orbbec_sdk_ok:
    print("✅ OrbbecSDK Python installato")
else:
    print("⚠️ OrbbecSDK Python non trovato (opzionale)")

print("\n" + "=" * 80)
print("5. OBJECT DETECTION E AI")
print("=" * 80)

# YOLOv8
yolov8_ok, yolov8_version = check_python_package('ultralytics')
if yolov8_ok:
    print(f"✅ YOLOv8 (ultralytics) installato ({yolov8_version})")
else:
    print("❌ YOLOv8 NON installato (pip install ultralytics)")

# OpenCV
opencv_ok, opencv_version = check_python_package('cv2', 'cv2')
if opencv_ok:
    print(f"✅ OpenCV installato ({opencv_version})")
else:
    print("❌ OpenCV NON installato (pip install opencv-python)")

# Open3D
open3d_ok, open3d_version = check_python_package('open3d')
if open3d_ok:
    print(f"✅ Open3D installato ({open3d_version})")
else:
    print("⚠️ Open3D NON installato (pip install open3d)")

# Isaac ROS
isaac_ok, isaac_list = check_ros2_package('isaac')
if isaac_ok:
    print(f"✅ Isaac ROS installato")
    for pkg in isaac_list[:3]:
        print(f"   - {pkg}")
else:
    print("⚠️ Isaac ROS non trovato (integrato in AI Accelerator 1.1)")

print("\n" + "=" * 80)
print("6. MOTION PLANNING")
print("=" * 80)

# MoveIt2
moveit_ok, moveit_list = check_ros2_package('moveit')
if moveit_ok:
    print(f"✅ MoveIt2 installato ({len(moveit_list)} pacchetti)")
    for pkg in moveit_list[:5]:
        print(f"   - {pkg}")
else:
    print("❌ MoveIt2 NON installato (sudo apt install ros-humble-moveit)")

# ros2_control
ros2_control_ok, ros2_control_list = check_ros2_package('ros2_control')
if ros2_control_ok:
    print(f"✅ ros2_control installato")
else:
    print("⚠️ ros2_control non trovato (incluso nel driver UR)")

print("\n" + "=" * 80)
print("7. COMPONENTI CUSTOM")
print("=" * 80)

# Web Interface
web_interface = os.path.expanduser('~/MekoAiAccelerator/remote_ur_control/web_interface.py')
if check_file(web_interface):
    print(f"✅ Web Interface presente")
else:
    print(f"❌ Web Interface NON presente")

# ROS2 Bridge
ros2_bridge = os.path.expanduser('~/MekoAiAccelerator/ros2_bridge_fixed.py')
if check_file(ros2_bridge):
    print(f"✅ ROS2 Bridge presente")
else:
    print(f"❌ ROS2 Bridge NON presente")

# Remote UR Controller
remote_controller = os.path.expanduser('~/MekoAiAccelerator/remote_ur_control/remote_ur_controller.py')
if check_file(remote_controller):
    print(f"✅ Remote UR Controller presente")
else:
    print(f"❌ Remote UR Controller NON presente")

print("\n" + "=" * 80)
print("RIEPILOGO")
print("=" * 80)

# Conta componenti
components = {
    'Base': ['Ubuntu 22.04', 'Python 3.10+', 'ROS2 Humble'],
    'UR Driver': ['UR ROS2 Driver', 'ur_rtde'],
    'MuJoCo': ['MuJoCo', 'MuJoCo Menagerie'],
    'Orbbec': ['OrbbecSDK ROS2'],
    'AI': ['YOLOv8', 'OpenCV', 'Open3D'],
    'Planning': ['MoveIt2']
}

print("\n✅ Componenti installati e pronti")
print("❌ Componenti mancanti da installare")
print("⚠️ Componenti opzionali o da verificare")
print()












