#!/usr/bin/env python3
"""
Controllo robot UR5e con joystick fisico
Supporta joystick USB standard (gamepad)
"""

import socket
import time
import sys

# Prova a importare pygame per joystick
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("⚠️  Pygame non installato. Installare: pip install pygame")

ROBOT_IP = "192.168.10.194"
PRIMARY_PORT = 30001

# Velocità movimento
MAX_VELOCITY = 0.1  # rad/s
MAX_LINEAR_VEL = 0.05  # m/s

def send_urscript(script):
    """Invia comando URScript al robot"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        sock.connect((ROBOT_IP, PRIMARY_PORT))
        sock.send(script.encode() + b"\n")
        time.sleep(0.05)
        sock.close()
        return True
    except Exception as e:
        print(f"Errore invio: {e}")
        return False

def init_joystick():
    """Inizializza joystick"""
    if not PYGAME_AVAILABLE:
        return None
    
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("❌ Nessun joystick trovato!")
        return None
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"✅ Joystick trovato: {joystick.get_name()}")
    print(f"   Assi: {joystick.get_numaxes()}")
    print(f"   Pulsanti: {joystick.get_numbuttons()}")
    
    return joystick

def get_joystick_input(joystick):
    """Leggi input joystick"""
    pygame.event.pump()
    
    # Assi joystick (normalizzati -1.0 a 1.0)
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    
    return axes, buttons

def create_move_script(linear_vel, angular_vel):
    """Crea script URScript per movimento"""
    # linear_vel: [x, y, z] in m/s
    # angular_vel: [rx, ry, rz] in rad/s
    
    script = f"""
def joystick_move():
    # Velocità lineare TCP
    linear = [{linear_vel[0]}, {linear_vel[1]}, {linear_vel[2]}, 0.0, 0.0, 0.0]
    
    # Velocità angolare TCP
    angular = [{angular_vel[0]}, {angular_vel[1]}, {angular_vel[2]}, 0.0, 0.0, 0.0]
    
    # Movimento velocità
    speedl(linear, a=0.1, t=0.1)
end

joystick_move()
"""
    return script

def main():
    print("=" * 60)
    print("CONTROLLO ROBOT UR5e CON JOYSTICK FISICO")
    print("=" * 60)
    print()
    
    # Inizializza joystick
    joystick = init_joystick()
    if not joystick:
        print("❌ Impossibile inizializzare joystick!")
        print("   Verifica che joystick sia collegato")
        if not PYGAME_AVAILABLE:
            print("   Installa pygame: pip install pygame")
        return
    
    print()
    print("Controlli:")
    print("  Left Stick: Movimento X/Y TCP")
    print("  Right Stick: Movimento Z/RX TCP")
    print("  Trigger: Velocità")
    print("  START: Esci")
    print()
    print("⚠️  Il robot si muoverà in base al joystick!")
    print("   Premi START per fermare")
    print()
    
    input("Premi ENTER per iniziare...")
    
    print("Controllo attivo...")
    print("Premi START sul joystick per fermare")
    print()
    
    running = True
    last_send_time = time.time()
    send_interval = 0.1  # Invia comando ogni 100ms
    
    try:
        while running:
            axes, buttons = get_joystick_input(joystick)
            
            # Pulsante START (solitamente pulsante 7 o 9)
            if len(buttons) > 7 and buttons[7]:
                print("STOP richiesto")
                running = False
                break
            
            # Mappa assi joystick
            # Left stick X/Y -> movimento TCP X/Y
            # Right stick Y -> movimento TCP Z
            # Right stick X -> rotazione TCP RX
            
            if len(axes) >= 4:
                # Deadzone
                deadzone = 0.1
                
                # Movimento lineare
                x_vel = axes[0] * MAX_LINEAR_VEL if abs(axes[0]) > deadzone else 0.0
                y_vel = -axes[1] * MAX_LINEAR_VEL if abs(axes[1]) > deadzone else 0.0
                z_vel = -axes[3] * MAX_LINEAR_VEL if abs(axes[3]) > deadzone else 0.0
                
                # Rotazione
                rx_vel = axes[2] * MAX_VELOCITY if abs(axes[2]) > deadzone else 0.0
                
                linear_vel = [x_vel, y_vel, z_vel]
                angular_vel = [rx_vel, 0.0, 0.0]
                
                # Invia comando solo se c'è movimento
                if any(abs(v) > 0.01 for v in linear_vel + angular_vel):
                    if time.time() - last_send_time >= send_interval:
                        script = create_move_script(linear_vel, angular_vel)
                        send_urscript(script)
                        last_send_time = time.time()
                        
                        # Mostra velocità
                        print(f"\rVel: X={x_vel:.3f} Y={y_vel:.3f} Z={z_vel:.3f} RX={rx_vel:.3f}", end="", flush=True)
                else:
                    # Stop se nessun movimento
                    if time.time() - last_send_time >= send_interval:
                        script = create_move_script([0, 0, 0], [0, 0, 0])
                        send_urscript(script)
                        last_send_time = time.time()
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\nInterrotto da utente")
    finally:
        # Stop robot
        print("\nFermando robot...")
        script = create_move_script([0, 0, 0], [0, 0, 0])
        send_urscript(script)
        time.sleep(0.5)
        
        if joystick:
            joystick.quit()
        pygame.quit()
        
        print("✅ Controllo terminato")

if __name__ == "__main__":
    main()











