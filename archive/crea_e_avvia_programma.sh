#!/bin/bash
# Crea programma direttamente sul robot via SSH e lo avvia

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"

echo "=========================================="
echo "CREAZIONE E AVVIO PROGRAMMA SUL ROBOT"
echo "=========================================="
echo ""

# Contenuto programma minimale in formato URScript
PROGRAM_CONTENT='def main():
  while True:
    sync()
  end
end

main()'

echo "1. Creazione programma via SSH..."
ssh -o StrictHostKeyChecking=accept-new ${ROBOT_USER}@${ROBOT_IP} << 'ENDSSH'
# Crea directory se non esiste
mkdir -p /programs

# Crea file script minimale
cat > /tmp/remote_control_minimal.script << 'SCRIPT'
def main():
  while True:
    sync()
  end
end

main()
SCRIPT

# Copia nella directory programs
cp /tmp/remote_control_minimal.script /programs/remote_control_minimal.script
chmod 644 /programs/remote_control_minimal.script

echo "✅ Script creato: /programs/remote_control_minimal.script"
echo ""
echo "NOTA: Per caricare il programma sul Teach Pendant:"
echo "1. Vai su Program → Load"
echo "2. Cerca 'remote_control_minimal.script'"
echo "3. Caricalo e salvalo come .urp"
echo "4. Metti in REMOTE e premi PLAY"
ENDSSH

echo ""
echo "2. Tentativo avvio via Dashboard Server..."
python3 << 'PYTHON'
import socket
import time

ROBOT_IP = "192.168.10.194"
PORT = 29999

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((ROBOT_IP, PORT))
    sock.recv(1024)  # Welcome
    
    # Verifica stato
    print("   Stato robot:")
    sock.sendall(b"robotmode\n")
    time.sleep(0.2)
    robotmode = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      {robotmode}")
    
    sock.sendall(b"programState\n")
    time.sleep(0.2)
    program_state = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      {program_state}")
    
    # Se STOPPED, prova a fare play
    if "STOPPED" in program_state:
        print("\n   Tentativo avvio programma...")
        sock.sendall(b"play\n")
        time.sleep(0.5)
        response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        print(f"      Risposta: {response}")
        time.sleep(2)
        
        # Verifica nuovo stato
        sock.sendall(b"programState\n")
        time.sleep(0.2)
        new_state = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        print(f"      Nuovo stato: {new_state}")
        
        if "PLAYING" in new_state:
            print("\n   ✅ Programma avviato!")
        else:
            print("\n   ⚠️  Programma non avviato automaticamente")
            print("   Devi avviarlo manualmente sul Teach Pendant")
    elif "PLAYING" in program_state:
        print("\n   ✅ Programma già in PLAYING!")
    else:
        print("\n   ⚠️  Stato programma: " + program_state)
    
    sock.close()
    
except Exception as e:
    print(f"   ❌ Errore: {e}")
PYTHON

echo ""
echo "=========================================="
echo "RIEPILOGO"
echo "=========================================="
echo ""
echo "Se il programma non è in PLAYING:"
echo "1. Sul Teach Pendant: Program → Load"
echo "2. Carica: remote_control_minimal.script"
echo "3. Salvalo come .urp"
echo "4. Metti robot in REMOTE"
echo "5. Premi PLAY"
echo ""

