#!/usr/bin/env python3
"""
Script robusto per switch controller ROS2 - verifica stato PRIMA di fare switch.
Evita errori quando il controller è già attivo o quando si cerca di deattivare controller inesistenti.
"""
import sys
import os
import subprocess
import time
import re

# CRITICAL: Setup ROS2 environment BEFORE importing rclpy
def setup_ros2_environment():
    """Setup ROS2 environment variables before importing rclpy."""
    if 'ROS_DISTRO' in os.environ:
        # Already sourced, just ensure LD_LIBRARY_PATH is set
        ros_lib = '/opt/ros/humble/lib'
        current_ld = os.environ.get('LD_LIBRARY_PATH', '')
        if ros_lib not in current_ld:
            os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib
        return
    
    # Not sourced - try to get environment from ROS2 setup script
    ros_setup = '/opt/ros/humble/setup.bash'
    if not os.path.exists(ros_setup):
        # Fallback: just set common ROS2 paths
        ros_lib = '/opt/ros/humble/lib'
        current_ld = os.environ.get('LD_LIBRARY_PATH', '')
        if ros_lib not in current_ld:
            os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib
        return
    
    # Source ROS2 setup and extract environment variables
    try:
        # Use env command to get all environment variables after sourcing
        cmd = f'bash -c "source {ros_setup} && env"'
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            # Parse environment variables
            for line in result.stdout.strip().split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    # Set critical environment variables
                    if key in ['LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION', 'CMAKE_PREFIX_PATH']:
                        if key == 'LD_LIBRARY_PATH':
                            # Merge with existing LD_LIBRARY_PATH
                            current = os.environ.get('LD_LIBRARY_PATH', '')
                            if current:
                                os.environ[key] = f'{value}:{current}'
                            else:
                                os.environ[key] = value
                        elif key == 'PYTHONPATH':
                            # Merge with existing PYTHONPATH
                            current = os.environ.get('PYTHONPATH', '')
                            if current:
                                os.environ[key] = f'{value}:{current}'
                            else:
                                os.environ[key] = value
                        else:
                            os.environ[key] = value
            
            # Ensure ROS2 lib is in LD_LIBRARY_PATH (critical for librcl_action.so)
            ros_lib = '/opt/ros/humble/lib'
            current_ld = os.environ.get('LD_LIBRARY_PATH', '')
            if ros_lib not in current_ld:
                os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib
        else:
            # Fallback: set common paths
            ros_lib = '/opt/ros/humble/lib'
            current_ld = os.environ.get('LD_LIBRARY_PATH', '')
            if ros_lib not in current_ld:
                os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib
    except Exception as e:
        # Fallback: set common paths
        ros_lib = '/opt/ros/humble/lib'
        current_ld = os.environ.get('LD_LIBRARY_PATH', '')
        if ros_lib not in current_ld:
            os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib

# Setup ROS2 environment BEFORE importing rclpy
setup_ros2_environment()

def get_controller_status(ros2_setup):
    """Ottiene lo stato corrente dei controller."""
    try:
        cmd = f"""{ros2_setup} && ros2 control list_controllers 2>&1"""
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode != 0:
            return None
        
        # Parse output
        controllers = {}
        lines = result.stdout.split('\n')
        for line in lines:
            line = line.strip()
            if not line or 'name' in line.lower() or 'state' in line.lower():
                continue
            
            # Format: "controller_name [unconfigured|inactive|active]"
            parts = line.split()
            if len(parts) >= 2:
                name = parts[0]
                state = parts[1]
                controllers[name] = state
        
        return controllers
    except Exception as e:
        print(f"ERROR: Errore verifica stato: {e}", file=sys.stderr)
        return None

def main():
    print("[DEBUG] switch_controller.py avviato", file=sys.stderr)
    print(f"[DEBUG] Argomenti: {sys.argv}", file=sys.stderr)
    
    if len(sys.argv) < 3:
        print("ERROR: Usage: switch_controller.py <activate> <deactivate>", file=sys.stderr)
        sys.exit(1)
    
    activate_controller = sys.argv[1]
    deactivate_controller = sys.argv[2]
    
    print(f"[DEBUG] Attivazione: {activate_controller}, Deattivazione: {deactivate_controller}", file=sys.stderr)
    print(f"[DEBUG] LD_LIBRARY_PATH={os.environ.get('LD_LIBRARY_PATH', 'NOT SET')}", file=sys.stderr)
    print(f"[DEBUG] PYTHONPATH={os.environ.get('PYTHONPATH', 'NOT SET')}", file=sys.stderr)
    print(f"[DEBUG] ROS_DISTRO={os.environ.get('ROS_DISTRO', 'NOT SET')}", file=sys.stderr)
    
    # Setup ROS2
    ros2_setup = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
    
    # PRIMA: Verifica stato corrente
    print("[DEBUG] Verifica stato corrente controller...", file=sys.stderr)
    current_status = get_controller_status(ros2_setup)
    
    if current_status is None:
        print("WARNING: Impossibile verificare stato controller, procedo comunque...", file=sys.stderr)
    else:
        print(f"✅ Stato corrente: {current_status}", file=sys.stderr)
        
        # Se il controller da attivare è già attivo, ritorna successo
        if activate_controller in current_status:
            if current_status[activate_controller] == 'active':
                print(f"OK: Controller {activate_controller} già attivo - nessuna azione necessaria")
                sys.exit(0)
        
        # Se il controller da deattivare non esiste o non è attivo, non provare a deattivarlo
        if deactivate_controller:
            if deactivate_controller not in current_status:
                print(f"INFO: Controller {deactivate_controller} non esiste - salto deattivazione", file=sys.stderr)
                deactivate_controller = None  # Non provare a deattivarlo
            elif current_status[deactivate_controller] != 'active':
                print(f"INFO: Controller {deactivate_controller} non attivo - salto deattivazione", file=sys.stderr)
                deactivate_controller = None  # Non provare a deattivarlo
    
    # Se non c'è nulla da fare, ritorna successo
    if not deactivate_controller and activate_controller in current_status and current_status.get(activate_controller) == 'active':
        print(f"OK: Controller {activate_controller} già attivo")
        sys.exit(0)
    
    # Metodo 1: Usa ros2 service switch_controller con strictness=0 (permette switch parziali)
    print(f"[DEBUG] Tentativo switch controller con rclpy...", file=sys.stderr)
    
    try:
        # Import rclpy AFTER environment is set up
        print(f"[DEBUG] Tentativo import rclpy...", file=sys.stderr)
        try:
            import rclpy
            from controller_manager_msgs.srv import SwitchController
            print(f"[DEBUG] rclpy importato con successo", file=sys.stderr)
        except ImportError as import_err:
            print(f"ERROR: Impossibile importare rclpy: {import_err}", file=sys.stderr)
            print(f"   LD_LIBRARY_PATH={os.environ.get('LD_LIBRARY_PATH', 'NOT SET')}", file=sys.stderr)
            print(f"   PYTHONPATH={os.environ.get('PYTHONPATH', 'NOT SET')}", file=sys.stderr)
            print(f"   Verifica che ROS2 sia installato: sudo apt install ros-humble-desktop", file=sys.stderr)
            raise
        
        # Inizializza ROS2
        print(f"[DEBUG] Inizializzazione ROS2...", file=sys.stderr)
        try:
            if not rclpy.ok():
                rclpy.init()
            print(f"[DEBUG] ROS2 inizializzato", file=sys.stderr)
        except Exception as init_err:
            print(f"ERROR: Impossibile inizializzare ROS2: {init_err}", file=sys.stderr)
            print(f"   LD_LIBRARY_PATH={os.environ.get('LD_LIBRARY_PATH', 'NOT SET')}", file=sys.stderr)
            print(f"   Verifica che ROS2 sia installato e configurato", file=sys.stderr)
            raise
        
        # Crea nodo
        print(f"[DEBUG] Creazione nodo ROS2...", file=sys.stderr)
        from rclpy.node import Node
        node_name = f'controller_switcher_{int(time.time())}'
        node = Node(node_name)
        print(f"[DEBUG] Nodo creato: {node_name}", file=sys.stderr)
        
        # Prova servizio switch_controller
        service_name = '/controller_manager/switch_controller'
        print(f"[DEBUG] Creazione client per servizio: {service_name}", file=sys.stderr)
        client = node.create_client(SwitchController, service_name)
        
        # Attendi servizio (timeout aumentato)
        print(f"[DEBUG] Attesa servizio (timeout 20s)...", file=sys.stderr)
        if client.wait_for_service(timeout_sec=20.0):
            print(f"[DEBUG] Servizio disponibile!", file=sys.stderr)
            print(f"✅ Servizio {service_name} disponibile", file=sys.stderr)
            
            req = SwitchController.Request()
            req.activate_controllers = [activate_controller] if activate_controller else []
            req.deactivate_controllers = [deactivate_controller] if deactivate_controller else []
            req.strictness = 0  # BEST_EFFORT - permette switch parziali
            req.activate_asap = True
            # Timeout deve essere un messaggio builtin_interfaces/Duration, non rclpy.duration.Duration
            from builtin_interfaces.msg import Duration
            req.timeout = Duration()
            req.timeout.sec = 5
            req.timeout.nanosec = 0
            
            print(f"🔄 Richiesta switch: activate={req.activate_controllers}, deactivate={req.deactivate_controllers}, strictness=0", file=sys.stderr)
            
            # Usa chiamata asincrona con timeout
            future = client.call_async(req)
            
            # Attendi risposta con timeout (aumentato per dare più tempo)
            print(f"[DEBUG] Attesa risposta servizio (timeout 20s)...", file=sys.stderr)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
            
            if future.done():
                try:
                    response = future.result()
                    if response and response.ok:
                        # Verifica che il controller sia effettivamente attivo
                        print(f"[DEBUG] Risposta servizio: ok={response.ok}", file=sys.stderr)
                        # Attendi un momento per permettere al controller di attivarsi
                        time.sleep(0.5)
                        
                        # Verifica stato finale
                        from controller_manager_msgs.srv import ListControllers
                        list_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
                        if list_client.wait_for_service(timeout_sec=2.0):
                            list_req = ListControllers.Request()
                            list_future = list_client.call_async(list_req)
                            rclpy.spin_until_future_complete(node, list_future, timeout_sec=2.0)
                            if list_future.done():
                                list_response = list_future.result()
                                if list_response:
                                    for ctrl in list_response.controller:
                                        if ctrl.name == activate_controller:
                                            if ctrl.state == 'active':
                                                print(f"OK: Controller {activate_controller} attivato con successo")
                                                node.destroy_node()
                                                rclpy.shutdown()
                                                sys.exit(0)
                                            else:
                                                print(f"WARNING: Controller {activate_controller} risulta in stato '{ctrl.state}' invece di 'active'", file=sys.stderr)
                                                break
                        
                        # Se la verifica non funziona, considera comunque successo se response.ok
                        print(f"OK: Controller {activate_controller} attivato con successo (risposta servizio: ok)")
                        node.destroy_node()
                        rclpy.shutdown()
                        sys.exit(0)
                    else:
                        error_msg = f"Switch controller fallito: {response}" if response else "Nessuna risposta dal servizio"
                        print(f"WARNING: {error_msg}", file=sys.stderr)
                except Exception as e:
                    print(f"WARNING: Errore lettura risposta: {e}", file=sys.stderr)
            else:
                print(f"ERROR: Timeout attesa risposta servizio switch_controller dopo 20s", file=sys.stderr)
                print(f"[DEBUG] Il servizio non ha risposto entro 20 secondi", file=sys.stderr)
            
            node.destroy_node()
            rclpy.shutdown()
        else:
            print(f"ERROR: Servizio {service_name} non disponibile dopo 20s", file=sys.stderr)
            print(f"[DEBUG] Verifica driver ROS2...", file=sys.stderr)
            # Verifica se il driver è attivo
            try:
                check_cmd = "pgrep -f ur_ros2_control_node"
                check_result = subprocess.run(['bash', '-c', check_cmd], capture_output=True, text=True, timeout=2)
                if check_result.returncode == 0:
                    print(f"[DEBUG] Driver ROS2 attivo (PID: {check_result.stdout.strip()})", file=sys.stderr)
                else:
                    print(f"[DEBUG] Driver ROS2 NON attivo", file=sys.stderr)
            except:
                pass
            print(f"   Verifica che:", file=sys.stderr)
            print(f"   1. Il driver ROS2 sia avviato: pgrep -f ur_ros2_control_node", file=sys.stderr)
            print(f"   2. Il controller_manager sia attivo: ros2 service list | grep controller_manager", file=sys.stderr)
            client.destroy()
            node.destroy_node()
            rclpy.shutdown()
    except ImportError as e:
        print(f"ERROR: rclpy non disponibile: {e}", file=sys.stderr)
        print(f"   Installa ROS2: sudo apt install ros-humble-desktop", file=sys.stderr)
    except Exception as e:
        print(f"ERROR: Errore Python rclpy: {e}", file=sys.stderr)
        import traceback
        print(f"   Traceback: {traceback.format_exc()}", file=sys.stderr)
        try:
            rclpy.shutdown()
        except:
            pass
    
    # Metodo 2: Usa spawner/unspawner
    print(f"⏳ Tentativo con spawner/unspawner...", file=sys.stderr)
    
    # Deattiva solo se necessario e se esiste
    if deactivate_controller:
        try:
            unspawn_cmd = f"""{ros2_setup} && timeout 5 ros2 run controller_manager unspawner {deactivate_controller} 2>&1 || true"""
            unspawn_result = subprocess.run(
                ['bash', '-c', unspawn_cmd],
                capture_output=True,
                text=True,
                timeout=7
            )
            # Ignora errori - se non esiste, va bene
        except:
            pass
    
    # Attiva controller
    spawn_cmd = f"""{ros2_setup} && ros2 run controller_manager spawner {activate_controller} 2>&1"""
    spawn_result = subprocess.run(
        ['bash', '-c', spawn_cmd],
        capture_output=True,
        text=True,
        timeout=15
    )
    
    if spawn_result.returncode == 0:
        print(f"OK: Controller {activate_controller} attivato con spawner")
        sys.exit(0)
    else:
        error_msg = spawn_result.stderr.strip() or spawn_result.stdout.strip()
        # Se dice "already active", considera successo
        if 'already active' in error_msg.lower() or 'already started' in error_msg.lower():
            print(f"OK: Controller {activate_controller} già attivo")
            sys.exit(0)
        print(f"WARNING: spawner fallito: {error_msg}", file=sys.stderr)
    
    # Verifica finale stato
    print(f"⏳ Verifica finale stato controller...", file=sys.stderr)
    final_status = get_controller_status(ros2_setup)
    
    if final_status and activate_controller in final_status:
        if final_status[activate_controller] == 'active':
            print(f"OK: Controller {activate_controller} risulta attivo (verifica finale)")
            sys.exit(0)
    
    # Se arriviamo qui, tutti i metodi sono falliti
    print(f"ERROR: Impossibile attivare controller {activate_controller}", file=sys.stderr)
    print(f"   Verifica che:", file=sys.stderr)
    print(f"   1. Il driver ROS2 sia avviato: pgrep -f ur_ros2_control_node", file=sys.stderr)
    print(f"   2. Il controller_manager sia attivo: ros2 service list | grep controller_manager", file=sys.stderr)
    print(f"   3. Il robot sia in Remote Control sul Teach Pendant", file=sys.stderr)
    
    sys.exit(1)

if __name__ == '__main__':
    main()
