#!/usr/bin/env python3
"""
Test connessione RTDE al robot UR5e usando libreria ufficiale UR
"""

import rtde.rtde as rtde

ROBOT_IP = "192.168.10.194"
RTDE_PORT = 30004

print("=" * 60)
print("TEST CONNESSIONE RTDE - UR5e")
print("=" * 60)
print(f"Robot IP: {ROBOT_IP}")
print(f"Porta RTDE: {RTDE_PORT}")
print()

try:
    print("1. Creazione client RTDE...")
    rtde_client = rtde.RTDE(ROBOT_IP, RTDE_PORT)
    print("   ✅ Client creato")
    
    print("\n2. Connessione al robot...")
    rtde_client.connect()
    print("   ✅ Connesso!")
    
    print("\n3. Setup input/output...")
    # Setup input (comandi da inviare)
    rtde_client.send_input_setup(['target_q'], rtde.RTDE.DataType.VECTOR6D, 1)
    
    # Setup output (dati da ricevere)
    rtde_client.send_output_setup(['actual_q', 'actual_qd', 'robot_status'], 
                                   [rtde.RTDE.DataType.VECTOR6D, 
                                    rtde.RTDE.DataType.VECTOR6D,
                                    rtde.RTDE.DataType.UINT32], 1)
    
    print("   ✅ Setup completato")
    
    print("\n4. Avvio comunicazione...")
    rtde_client.send_start()
    print("   ✅ Comunicazione avviata")
    
    print("\n5. Ricezione dati dal robot...")
    state = rtde_client.receive()
    if state:
        print("   ✅ Dati ricevuti!")
        print(f"   Joint positions: {state.actual_q}")
        print(f"   Joint velocities: {state.actual_qd}")
        print(f"   Robot status: {state.robot_status}")
    else:
        print("   ⚠️  Nessun dato ricevuto")
    
    print("\n6. Pausa e disconnessione...")
    rtde_client.send_pause()
    rtde_client.disconnect()
    print("   ✅ Disconnesso")
    
    print("\n" + "=" * 60)
    print("✅ TEST COMPLETATO CON SUCCESSO!")
    print("=" * 60)
    
except Exception as e:
    print(f"\n❌ ERRORE: {e}")
    import traceback
    traceback.print_exc()
    exit(1)





