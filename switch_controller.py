#!/usr/bin/env python3
"""
Script per switch controller ROS2 - usa spawner/unspawner ufficiali dalla documentazione.
"""
import sys
import os
import subprocess
import time

def main():
    if len(sys.argv) < 3:
        print("ERROR: Usage: switch_controller.py <activate> <deactivate>", file=sys.stderr)
        sys.exit(1)
    
    activate_controller = sys.argv[1]
    deactivate_controller = sys.argv[2]
    
    # Setup ROS2
    ros2_setup = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
    
    # Metodo 1: Usa spawner/unspawner ufficiali (raccomandato dalla documentazione)
    print(f"⏳ Tentativo con spawner/unspawner (metodo ufficiale)...", file=sys.stderr)
    
    # Prima ferma e rimuovi il controller da deattivare
    if deactivate_controller:
        unspawn_cmd = f"""{ros2_setup} && ros2 run controller_manager unspawner {deactivate_controller} 2>&1"""
        unspawn_result = subprocess.run(
            ['bash', '-c', unspawn_cmd],
            capture_output=True,
            text=True,
            timeout=10
        )
        # Ignora errori se il controller non è attivo
    
    # Usa spawner per caricare e avviare il controller
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
        print(f"WARNING: spawner fallito: {error_msg}", file=sys.stderr)
    
    # Metodo 2: Usa ros2 control CLI (se disponibile)
    print(f"⏳ Tentativo con ros2 control CLI...", file=sys.stderr)
    
    check_cmd = f"""{ros2_setup} && ros2 control --help 2>&1 | head -1"""
    check_result = subprocess.run(
        ['bash', '-c', check_cmd],
        capture_output=True,
        text=True,
        timeout=5
    )
    
    if check_result.returncode == 0 and 'control' in check_result.stdout.lower():
        # Usa ros2 control switch_controllers
        cmd = f"""{ros2_setup} && ros2 control switch_controllers \\
            --activate {activate_controller} \\
            --deactivate {deactivate_controller} \\
            --strictness 1"""
        
        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True,
            text=True,
            timeout=15
        )
        
        if result.returncode == 0:
            print(f"OK: Controller {activate_controller} attivato con ros2 control CLI")
            sys.exit(0)
        else:
            print(f"WARNING: ros2 control CLI fallito: {result.stderr.strip()}", file=sys.stderr)
    
    # Metodo 3: Usa ros2 control load_controller e start_controller
    print(f"⏳ Tentativo con ros2 control load/start...", file=sys.stderr)
    
    # Verifica se il controller è già caricato
    list_cmd = f"""{ros2_setup} && ros2 control list_controllers 2>&1"""
    list_result = subprocess.run(
        ['bash', '-c', list_cmd],
        capture_output=True,
        text=True,
        timeout=10
    )
    
    controller_loaded = False
    controller_active = False
    
    if list_result.returncode == 0:
        if activate_controller in list_result.stdout:
            controller_loaded = True
            if 'active' in list_result.stdout.split(activate_controller)[1].split('\n')[0]:
                controller_active = True
    
    # Carica il controller solo se non è già caricato
    if not controller_loaded:
        load_cmd = f"""{ros2_setup} && ros2 control load_controller {activate_controller} 2>&1"""
        load_result = subprocess.run(
            ['bash', '-c', load_cmd],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if load_result.returncode != 0:
            error_msg = load_result.stderr.strip() or load_result.stdout.strip()
            print(f"WARNING: load_controller fallito: {error_msg}", file=sys.stderr)
    else:
        print(f"✅ Controller {activate_controller} già caricato", file=sys.stderr)
    
    # Avvia il controller solo se non è già attivo
    if not controller_active:
        start_cmd = f"""{ros2_setup} && ros2 control start_controller {activate_controller} 2>&1"""
        start_result = subprocess.run(
            ['bash', '-c', start_cmd],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if start_result.returncode == 0:
            print(f"OK: Controller {activate_controller} attivato con ros2 control")
            sys.exit(0)
        else:
            error_msg = start_result.stderr.strip() or start_result.stdout.strip()
            print(f"WARNING: start_controller fallito: {error_msg}", file=sys.stderr)
            # Se dice "already active", consideralo successo
            if 'already active' in error_msg.lower() or 'already started' in error_msg.lower():
                print(f"OK: Controller {activate_controller} già attivo")
                sys.exit(0)
    else:
        print(f"OK: Controller {activate_controller} già attivo")
        sys.exit(0)
    
    # Metodo 4: Usa Python con rclpy (fallback)
    print(f"⏳ Tentativo con Python rclpy...", file=sys.stderr)
    
    try:
        # Aggiungi path ROS2
        ros2_install = "/opt/ros/humble"
        if os.path.exists(ros2_install):
            python_path = os.path.join(ros2_install, "lib", "python3.10", "site-packages")
            if python_path not in sys.path:
                sys.path.insert(0, python_path)
        
        import rclpy
        from controller_manager_msgs.srv import SwitchController
        
        # Inizializza ROS2
        try:
            if not rclpy.ok():
                rclpy.init()
        except:
            rclpy.init()
        
        # Crea nodo con nome unico
        node_name = f'controller_switcher_{int(time.time())}'
        node = rclpy.create_node(node_name)
        
        # Prova entrambi i nomi di servizio possibili
        service_names = [
            '/controller_manager/switch_controller',
            '/controller_manager/switch_controllers',
        ]
        
        for service_name in service_names:
            client = node.create_client(SwitchController, service_name)
            
            if client.wait_for_service(timeout_sec=5.0):
                print(f"✅ Servizio {service_name} disponibile", file=sys.stderr)
                
                req = SwitchController.Request()
                req.activate_controllers = [activate_controller]
                req.deactivate_controllers = [deactivate_controller]
                req.strictness = 1
                
                response = client.call(req)
                
                node.destroy_node()
                rclpy.shutdown()
                
                if response.ok:
                    print(f"OK: Controller {activate_controller} attivato con Python")
                    sys.exit(0)
                else:
                    print(f"ERROR: Switch controller fallito: {response}", file=sys.stderr)
                    break
            else:
                client.destroy()
        
        node.destroy_node()
        rclpy.shutdown()
        
    except ImportError as e:
        print(f"ERROR: Import error: {e}", file=sys.stderr)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
    
    # Se arriviamo qui, tutti i metodi sono falliti
    print(f"ERROR: Impossibile attivare controller {activate_controller}", file=sys.stderr)
    print(f"   Verifica che:", file=sys.stderr)
    print(f"   1. Il driver ROS2 sia avviato: pgrep -f ur_ros2_control_node", file=sys.stderr)
    print(f"   2. Il controller_manager sia attivo: ros2 service list | grep controller_manager", file=sys.stderr)
    print(f"   3. Il robot sia in Remote Control sul Teach Pendant", file=sys.stderr)
    
    # Lista servizi disponibili per debug
    try:
        list_cmd = f"""{ros2_setup} && ros2 service list | grep controller"""
        list_result = subprocess.run(
            ['bash', '-c', list_cmd],
            capture_output=True,
            text=True,
            timeout=5
        )
        if list_result.returncode == 0 and list_result.stdout.strip():
            print(f"   Servizi controller_manager disponibili:", file=sys.stderr)
            for line in list_result.stdout.strip().split('\n'):
                print(f"     - {line}", file=sys.stderr)
    except:
        pass
    
    sys.exit(1)

if __name__ == '__main__':
    main()
