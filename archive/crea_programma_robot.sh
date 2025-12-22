#!/bin/bash
# Script per creare e avviare programma minimale sul robot

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"

echo "=========================================="
echo "CREAZIONE PROGRAMMA MINIMALE SUL ROBOT"
echo "=========================================="
echo ""

# 1. Crea script minimale
SCRIPT_CONTENT='def main():
  while True:
    sync()
  end
end

main()'

echo "1. Creazione script minimale..."
echo "$SCRIPT_CONTENT" > /tmp/remote_control.script

# 2. Trasferisci script al robot
echo "2. Trasferimento script al robot..."
scp /tmp/remote_control.script ${ROBOT_USER}@${ROBOT_IP}:/programs/remote_control.script 2>/dev/null || {
    echo "   ⚠️  Impossibile trasferire file direttamente"
    echo "   Uso Dashboard Server per creare programma..."
}

# 3. Usa Dashboard Server per caricare e avviare
echo "3. Configurazione via Dashboard Server..."
python3 << 'PYTHON'
import socket
import time

ROBOT_IP = "192.168.10.194"
PORT = 29999

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((ROBOT_IP, PORT))
    
    # Welcome
    welcome = sock.recv(1024)
    print(f"   Connesso: {welcome.decode('utf-8', errors='ignore').strip()}")
    
    # Verifica stato attuale
    print("\n   Stato attuale:")
    sock.sendall(b"robotmode\n")
    time.sleep(0.2)
    robotmode = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      Robot Mode: {robotmode}")
    
    sock.sendall(b"programState\n")
    time.sleep(0.2)
    program_state = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      Program State: {program_state}")
    
    # Se non in RUNNING, prova ad accendere
    if "POWER_OFF" in robotmode:
        print("\n   Robot spento - tentativo accensione...")
        sock.sendall(b"power on\n")
        time.sleep(0.2)
        response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        print(f"      Risposta: {response}")
        time.sleep(5)
    
    # Se in IDLE o POWER_ON, rilascia freni
    sock.sendall(b"robotmode\n")
    time.sleep(0.2)
    robotmode = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    
    if "POWER_ON" in robotmode or "IDLE" in robotmode:
        print("\n   Rilascio freni...")
        sock.sendall(b"brake release\n")
        time.sleep(0.2)
        response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        print(f"      Risposta: {response}")
        time.sleep(2)
    
    # Crea programma minimale via URScript
    print("\n   Creazione programma minimale...")
    print("   NOTA: Il programma deve essere creato manualmente sul Teach Pendant")
    print("   Oppure usa un programma esistente e mettilo in PLAYING")
    
    # Prova a vedere se c'è un programma caricato
    sock.sendall(b"get loaded program\n")
    time.sleep(0.2)
    loaded = sock.recv(1024).decode('utf-8', errors='ignore').strip()
    print(f"      Programma caricato: {loaded}")
    
    # Se c'è un programma, prova a farlo partire
    if "<unnamed>" not in loaded and "No program" not in loaded:
        print("\n   Tentativo avvio programma...")
        sock.sendall(b"play\n")
        time.sleep(0.5)
        response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        print(f"      Risposta: {response}")
        time.sleep(2)
        
        # Verifica stato finale
        sock.sendall(b"programState\n")
        time.sleep(0.2)
        final_state = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        print(f"      Stato finale: {final_state}")
        
        if "PLAYING" in final_state:
            print("\n   ✅ Programma in PLAYING!")
        else:
            print("\n   ⚠️  Programma non in PLAYING - avvialo manualmente sul Teach Pendant")
    else:
        print("\n   ⚠️  Nessun programma caricato")
        print("   Crea un programma minimale sul Teach Pendant con:")
        print("   def main():")
        print("     while True:")
        print("       sync()")
        print("     end")
        print("   end")
        print("   main()")
    
    sock.close()
    
except Exception as e:
    print(f"   ❌ Errore: {e}")
    import traceback
    traceback.print_exc()
PYTHON

echo ""
echo "=========================================="
echo "RIEPILOGO"
echo "=========================================="
echo ""
echo "Se il programma non è in PLAYING:"
echo "1. Sul Teach Pendant, crea un programma minimale"
echo "2. Metti robot in modalità REMOTE"
echo "3. Premi PLAY"
echo ""

