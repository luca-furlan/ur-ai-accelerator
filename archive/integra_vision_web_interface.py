#!/usr/bin/env python3
"""
Script per integrare Vision API nella web interface esistente

Uso:
    python3 integra_vision_web_interface.py

Questo script modifica automaticamente web_interface.py per aggiungere
gli endpoints vision senza dover modificare manualmente il codice.
"""

import os
import sys
import shutil
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WEB_INTERFACE_PATH = os.path.join(SCRIPT_DIR, 'remote_ur_control', 'web_interface.py')
BACKUP_PATH = WEB_INTERFACE_PATH + f'.backup.{datetime.now().strftime("%Y%m%d_%H%M%S")}'

# Codice da aggiungere all'inizio del file (dopo imports)
IMPORT_CODE = """
# ============================================
# VISION SYSTEM INTEGRATION
# ============================================
try:
    from .vision_web_api import add_vision_routes_to_app
    VISION_API_AVAILABLE = True
except ImportError:
    VISION_API_AVAILABLE = False
    print("⚠️ Vision Web API not available")
"""

# Codice da aggiungere dopo creazione app Flask
INIT_CODE = """
# ============================================
# Initialize Vision API
# ============================================
_vision_api = None
if VISION_API_AVAILABLE:
    try:
        _vision_api = add_vision_routes_to_app(app)
        app.logger.info("✅ Vision Web API initialized")
    except Exception as e:
        app.logger.error(f"❌ Failed to initialize Vision API: {e}")
"""


def backup_file(filepath):
    """Crea backup del file"""
    if os.path.exists(filepath):
        shutil.copy2(filepath, BACKUP_PATH)
        print(f"✅ Backup creato: {BACKUP_PATH}")
        return True
    return False


def check_already_integrated(filepath):
    """Verifica se vision API già integrata"""
    if not os.path.exists(filepath):
        return False
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    return 'vision_web_api' in content or 'VISION_API_AVAILABLE' in content


def integrate_vision_api():
    """Integra Vision API nella web interface"""
    
    print("=" * 60)
    print("INTEGRAZIONE VISION API → WEB INTERFACE")
    print("=" * 60)
    print()
    
    # Verifica esistenza file
    if not os.path.exists(WEB_INTERFACE_PATH):
        print(f"❌ File non trovato: {WEB_INTERFACE_PATH}")
        return False
    
    print(f"📄 File: {WEB_INTERFACE_PATH}")
    
    # Verifica se già integrato
    if check_already_integrated(WEB_INTERFACE_PATH):
        print("ℹ️  Vision API già integrata (skip)")
        return True
    
    # Backup
    if not backup_file(WEB_INTERFACE_PATH):
        print("❌ Errore creazione backup")
        return False
    
    # Leggi file
    print("📖 Lettura web_interface.py...")
    with open(WEB_INTERFACE_PATH, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # Trova posizioni per inserimento
    import_insert_line = None
    app_creation_line = None
    
    for i, line in enumerate(lines):
        # Trova dopo gli import (prima linea che inizia con "app = Flask")
        if 'app = Flask(' in line and app_creation_line is None:
            app_creation_line = i + 1
        
        # Trova fine imports (prima linea vuota dopo import)
        if line.strip().startswith('from flask import') and import_insert_line is None:
            import_insert_line = i + 1
    
    if import_insert_line is None or app_creation_line is None:
        print("❌ Impossibile trovare posizioni per integrazione")
        print(f"   Import line: {import_insert_line}")
        print(f"   App creation line: {app_creation_line}")
        return False
    
    print(f"   Insert import at line: {import_insert_line}")
    print(f"   Insert init at line: {app_creation_line}")
    
    # Inserisci codice
    print("✏️  Modifica file...")
    
    # Inserisci import
    lines.insert(import_insert_line, IMPORT_CODE + '\n')
    
    # Aggiorna app_creation_line dopo inserimento
    app_creation_line += IMPORT_CODE.count('\n') + 1
    
    # Inserisci init
    lines.insert(app_creation_line, INIT_CODE + '\n')
    
    # Scrivi file modificato
    print("💾 Salvataggio modifiche...")
    with open(WEB_INTERFACE_PATH, 'w', encoding='utf-8') as f:
        f.writelines(lines)
    
    print()
    print("=" * 60)
    print("✅ INTEGRAZIONE COMPLETATA")
    print("=" * 60)
    print()
    print("Nuovi endpoints disponibili:")
    print("  GET  /api/vision/status")
    print("  GET  /api/vision/detections")
    print("  POST /api/vision/start")
    print("  POST /api/vision/stop")
    print("  POST /api/vision/pick_target")
    print("  POST /api/vision/select_class")
    print("  GET  /api/vision/camera/info")
    print()
    print("Per testare:")
    print("  1. Avvia web interface:")
    print("     python -m remote_ur_control.web_interface")
    print()
    print("  2. Test endpoint:")
    print("     curl http://localhost:5000/api/vision/status")
    print()
    print(f"Backup originale: {BACKUP_PATH}")
    print("=" * 60)
    
    return True


def restore_backup():
    """Ripristina backup"""
    if not os.path.exists(BACKUP_PATH):
        print("❌ Backup non trovato")
        return False
    
    shutil.copy2(BACKUP_PATH, WEB_INTERFACE_PATH)
    print(f"✅ Ripristinato da: {BACKUP_PATH}")
    return True


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--restore':
        restore_backup()
    else:
        integrate_vision_api()


if __name__ == '__main__':
    main()




