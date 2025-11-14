"""
Test rapido per verificare che il server web risponda.
"""

import requests
import sys

def test_server(url="http://localhost:8080"):
    """Test se il server risponde."""
    try:
        print(f"Test connessione a {url}...")
        response = requests.get(url, timeout=5)
        print(f"[OK] Server risponde!")
        print(f"  Status: {response.status_code}")
        print(f"  Content length: {len(response.text)} bytes")
        print(f"  Content type: {response.headers.get('Content-Type', 'unknown')}")
        
        if "UR Remote Control" in response.text:
            print("[OK] Pagina contiene 'UR Remote Control' - tutto OK!")
        else:
            print("[ATTENZIONE] Pagina potrebbe non essere corretta")
            
        return True
    except requests.exceptions.ConnectionError:
        print(f"[ERR] Impossibile connettersi a {url}")
        print("  Verifica che il server sia avviato:")
        print("  python -m remote_ur_control.web_interface")
        return False
    except Exception as e:
        print(f"[ERR] Errore: {e}")
        return False

if __name__ == "__main__":
    # Test localhost
    local_ok = test_server("http://localhost:8080")
    
    # Test IP locale
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print(f"\nIP locale: {local_ip}")
    print(f"Prova anche: http://{local_ip}:8080")
    
    sys.exit(0 if local_ok else 1)

