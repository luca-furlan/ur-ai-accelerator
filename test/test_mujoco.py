#!/usr/bin/env python3
"""
Test MuJoCo - verifica installazione e modelli
"""

import os
import sys

def test_mujoco_import() -> tuple:
    """Test import MuJoCo"""
    try:
        import mujoco
        version = getattr(mujoco, '__version__', 'OK')
        return True, version
    except ImportError as e:
        return False, str(e)

def test_mujoco_viewer() -> bool:
    """Test se MuJoCo viewer è disponibile"""
    try:
        import mujoco.viewer
        return True
    except:
        return False

def check_mujoco_menagerie() -> dict:
    """Verifica presenza MuJoCo Menagerie e modelli"""
    results = {
        'menagerie_exists': False,
        'ur5e_exists': False,
        'ur10e_exists': False,
        'ur5e_path': None,
        'ur10e_path': None,
    }
    
    menagerie_path = os.path.expanduser('~/mujoco_menagerie')
    if os.path.isdir(menagerie_path):
        results['menagerie_exists'] = True
        
        ur5e_path = os.path.join(menagerie_path, 'universal_robots_ur5e')
        ur10e_path = os.path.join(menagerie_path, 'universal_robots_ur10e')
        
        if os.path.isdir(ur5e_path):
            results['ur5e_exists'] = True
            results['ur5e_path'] = ur5e_path
        
        if os.path.isdir(ur10e_path):
            results['ur10e_exists'] = True
            results['ur10e_path'] = ur10e_path
    
    return results

def test_load_model(model_path: str) -> bool:
    """Test caricamento modello MuJoCo"""
    try:
        import mujoco
        if os.path.isfile(model_path):
            model = mujoco.MjModel.from_xml_path(model_path)
            return model is not None
        return False
    except:
        return False

def main():
    print("=" * 60)
    print("TEST MUJOCO")
    print("=" * 60)
    
    # Test import
    print("\n1. Verifica installazione MuJoCo...")
    installed, version = test_mujoco_import()
    if installed:
        print(f"✅ MuJoCo installato (versione: {version})")
    else:
        print(f"❌ MuJoCo NON installato: {version}")
        print("   Installare: pip install mujoco")
        return
    
    # Test viewer
    print("\n2. Verifica MuJoCo viewer...")
    if test_mujoco_viewer():
        print("✅ MuJoCo viewer disponibile")
    else:
        print("⚠️ MuJoCo viewer non disponibile")
    
    # Verifica Menagerie
    print("\n3. Verifica MuJoCo Menagerie...")
    menagerie = check_mujoco_menagerie()
    if menagerie['menagerie_exists']:
        print("✅ MuJoCo Menagerie trovato")
        
        if menagerie['ur5e_exists']:
            print("✅ Modello UR5e presente")
            # Test caricamento
            scene_path = os.path.join(menagerie['ur5e_path'], 'scene.xml')
            if test_load_model(scene_path):
                print("   ✅ Modello UR5e caricabile")
            else:
                print("   ⚠️ Modello UR5e non caricabile")
        else:
            print("⚠️ Modello UR5e NON presente")
        
        if menagerie['ur10e_exists']:
            print("✅ Modello UR10e presente")
            scene_path = os.path.join(menagerie['ur10e_path'], 'scene.xml')
            if test_load_model(scene_path):
                print("   ✅ Modello UR10e caricabile")
            else:
                print("   ⚠️ Modello UR10e non caricabile")
        else:
            print("⚠️ Modello UR10e NON presente")
    else:
        print("❌ MuJoCo Menagerie NON trovato")
        print("   Installare: git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie")
    
    # Riepilogo
    print("\n" + "=" * 60)
    if installed and menagerie['menagerie_exists']:
        if menagerie['ur5e_exists'] or menagerie['ur10e_exists']:
            print("✅ MuJoCo installato e modelli disponibili")
        else:
            print("⚠️ MuJoCo installato ma modelli UR mancanti")
    elif installed:
        print("⚠️ MuJoCo installato ma Menagerie mancante")
    else:
        print("❌ MuJoCo non installato")
    print("\nNOTA: Per visualizzare un modello:")
    print("  python -m mujoco.viewer --mjcf ~/mujoco_menagerie/universal_robots_ur5e/scene.xml")
    print("=" * 60)

if __name__ == '__main__':
    main()












