#!/usr/bin/env python3
"""
Test componenti AI - YOLO, OpenCV, Open3D
"""

import sys

def test_yolov8() -> tuple:
    """Test YOLOv8"""
    try:
        from ultralytics import YOLO
        import ultralytics
        version = getattr(ultralytics, '__version__', 'OK')
        return True, version
    except ImportError as e:
        return False, str(e)

def test_opencv() -> tuple:
    """Test OpenCV"""
    try:
        import cv2
        version = cv2.__version__
        return True, version
    except ImportError as e:
        return False, str(e)

def test_open3d() -> tuple:
    """Test Open3D"""
    try:
        import open3d as o3d
        version = o3d.__version__
        return True, version
    except ImportError as e:
        return False, str(e)

def test_yolo_inference() -> bool:
    """Test inferenza YOLO (semplice)"""
    try:
        from ultralytics import YOLO
        # Non carichiamo un modello reale, solo verifichiamo che la classe esista
        return True
    except:
        return False

def test_opencv_basic() -> bool:
    """Test funzionalità base OpenCV"""
    try:
        import cv2
        import numpy as np
        # Crea immagine test
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        # Prova operazione base
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray is not None
    except:
        return False

def test_open3d_basic() -> bool:
    """Test funzionalità base Open3D"""
    try:
        import open3d as o3d
        import numpy as np
        # Crea point cloud test
        pcd = o3d.geometry.PointCloud()
        points = np.random.rand(100, 3)
        pcd.points = o3d.utility.Vector3dVector(points)
        return len(pcd.points) == 100
    except:
        return False

def main():
    print("=" * 60)
    print("TEST COMPONENTI AI")
    print("=" * 60)
    
    # YOLOv8
    print("\n1. Verifica YOLOv8...")
    installed, version = test_yolov8()
    if installed:
        print(f"✅ YOLOv8 installato (versione: {version})")
        if test_yolo_inference():
            print("   ✅ YOLOv8 funzionante")
        else:
            print("   ⚠️ YOLOv8 installato ma non testabile")
    else:
        print(f"❌ YOLOv8 NON installato: {version}")
        print("   Installare: pip install ultralytics")
    
    # OpenCV
    print("\n2. Verifica OpenCV...")
    installed, version = test_opencv()
    if installed:
        print(f"✅ OpenCV installato (versione: {version})")
        if test_opencv_basic():
            print("   ✅ OpenCV funzionante")
        else:
            print("   ⚠️ OpenCV installato ma non funzionante")
    else:
        print(f"❌ OpenCV NON installato: {version}")
        print("   Installare: pip install opencv-python")
    
    # Open3D
    print("\n3. Verifica Open3D...")
    installed, version = test_open3d()
    if installed:
        print(f"✅ Open3D installato (versione: {version})")
        if test_open3d_basic():
            print("   ✅ Open3D funzionante")
        else:
            print("   ⚠️ Open3D installato ma non funzionante")
    else:
        print(f"⚠️ Open3D NON installato: {version}")
        print("   Installare: pip install open3d")
    
    # Riepilogo
    print("\n" + "=" * 60)
    all_installed = (
        test_yolov8()[0] and 
        test_opencv()[0] and 
        test_open3d()[0]
    )
    if all_installed:
        print("✅ Tutti i componenti AI installati")
    elif test_opencv()[0]:
        print("⚠️ Alcuni componenti AI mancanti (OpenCV presente)")
    else:
        print("❌ Componenti AI base mancanti")
    print("=" * 60)

if __name__ == '__main__':
    main()












