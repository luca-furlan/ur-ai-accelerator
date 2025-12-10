#!/bin/bash
# Fix porta 50002 - External Control non attivo

set -e

echo "================================================================================"
echo "FIX PORTA 50002 - EXTERNAL CONTROL"
echo "================================================================================"
echo
echo "PROBLEMA IDENTIFICATO:"
echo "  - Programma in PLAYING: ✅"
echo "  - Porta 50002 (External Control): ❌ CHIUSA"
echo
echo "Il driver ROS2 si connette alla porta 50002 per il controllo."
echo "Se la porta è chiusa, il driver va in crash."
echo
echo "================================================================================"
echo "SOLUZIONE: CONFIGURA EXTERNAL CONTROL SUL TEACH PENDANT"
echo "================================================================================"
echo
echo "1. Vai sul TEACH PENDANT del robot"
echo
echo "2. Apri il programma 'remote_control.urp'"
echo
echo "3. Verifica il nodo 'External Control':"
echo "   - Deve essere presente nel programma"
echo "   - Deve essere configurato con:"
echo "     * IP Host: 192.168.10.191 (IP AI Accelerator)"
echo "     * Porta: 50002"
echo "     * Timeout: (default o 2.0)"
echo
echo "4. Se il nodo External Control NON è presente:"
echo "   a) Vai su: Installation → URCaps"
echo "   b) Verifica che 'External Control' sia installato"
echo "   c) Se non installato, installalo"
echo "   d) Torna al programma e aggiungi il nodo 'External Control'"
echo
echo "5. Salva il programma"
echo
echo "6. STOP e poi PLAY di nuovo il programma"
echo
echo "7. Verifica che la porta 50002 si apra:"
echo "   (Esegui questo script di nuovo dopo aver configurato)"
echo
echo "================================================================================"
read -p "Premi INVIO quando hai configurato External Control sul teach pendant..."

# Verifica di nuovo
echo
echo "Verifica configurazione..."
python3 << 'PYTHON'
import socket
import time

ROBOT_IP = "192.168.10.194"
MAX_RETRIES = 5

print("   Tentativo connessione porta 50002...")
for i in range(MAX_RETRIES):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((ROBOT_IP, 50002))
        sock.close()
        
        if result == 0:
            print(f"   ✅ Porta 50002 APERTA! (tentativo {i+1}/{MAX_RETRIES})")
            print("   ✅ External Control configurato correttamente")
            exit(0)
        else:
            print(f"   ⏳ Porta ancora chiusa... (tentativo {i+1}/{MAX_RETRIES})")
            time.sleep(1)
    except Exception as e:
        print(f"   ⚠️  Errore tentativo {i+1}: {e}")
        time.sleep(1)

print()
print("   ❌ Porta 50002 ancora CHIUSA dopo $MAX_RETRIES tentativi")
print()
print("   VERIFICA:")
print("   1. Il programma è in PLAYING?")
print("   2. Il nodo External Control è presente nel programma?")
print("   3. L'IP Host è configurato come 192.168.10.191?")
print("   4. La porta è configurata come 50002?")
print("   5. Hai fatto STOP e poi PLAY di nuovo dopo aver salvato?")
exit(1)
PYTHON

if [ $? -eq 0 ]; then
    echo
    echo "================================================================================"
    echo "✅ CONFIGURAZIONE CORRETTA!"
    echo "================================================================================"
    echo
    echo "Ora puoi avviare il driver ROS2:"
    echo
    echo "  ./AVVIA_DRIVER_UFFICIALE.sh"
    echo
else
    echo
    echo "================================================================================"
    echo "❌ CONFIGURAZIONE NON COMPLETA"
    echo "================================================================================"
    echo
    echo "Risolvi i problemi sopra indicati e riprova."
fi



