#!/usr/bin/env python3
"""Script per integrare Vision API in web_interface.py"""

import re
import sys
import os

def integrate_vision_api():
    web_interface_path = 'remote_ur_control/web_interface.py'
    
    if not os.path.exists(web_interface_path):
        print(f"❌ File non trovato: {web_interface_path}")
        return False
    
    with open(web_interface_path, 'r') as f:
        content = f.read()
    
    if 'vision_web_api' in content:
        print("✅ Vision API già integrata")
        return True
    
    # Backup
    import shutil
    backup_path = f"{web_interface_path}.backup"
    shutil.copy2(web_interface_path, backup_path)
    print(f"✅ Backup creato: {backup_path}")
    
    # Aggiungi import dopo "from flask import"
    import_pattern = r'(from flask import[^\n]+)'
    vision_import = r'''\1

# Vision API
try:
    from .vision_web_api import add_vision_routes_to_app
    VISION_API_AVAILABLE = True
except ImportError:
    VISION_API_AVAILABLE = False
    print("⚠️ Vision API non disponibile")'''
    
    content = re.sub(import_pattern, vision_import, content, count=1)
    
    # Aggiungi inizializzazione dopo "app = Flask"
    app_pattern = r'(app = Flask\([^\n]+)'
    vision_init = r'''\1

# Initialize Vision API
if VISION_API_AVAILABLE:
    try:
        vision_api = add_vision_routes_to_app(app)
        print("✅ Vision API integrata")
    except Exception as e:
        print(f"⚠️ Errore integrazione Vision API: {e}")'''
    
    content = re.sub(app_pattern, vision_init, content, count=1)
    
    with open(web_interface_path, 'w') as f:
        f.write(content)
    
    print("✅ Vision API integrata con successo")
    return True

if __name__ == '__main__':
    success = integrate_vision_api()
    sys.exit(0 if success else 1)




