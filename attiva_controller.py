#!/usr/bin/env python3
"""
Script Python robusto per attivare forward_velocity_controller.
Verifica lo stato PRIMA di fare switch per evitare errori.
"""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController, ListControllers

def get_controller_status(node, client_list):
    """Ottiene lo stato corrente dei controller."""
    try:
        if not client_list.wait_for_service(timeout_sec=3.0):
            return None
        
        request = ListControllers.Request()
        future = client_list.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        
        if future.result() is None:
            return None
        
        response = future.result()
        controllers = {}
        for controller in response.controller:
            controllers[controller.name] = controller.state
        return controllers
    except Exception as e:
        print(f"WARNING: Errore verifica stato: {e}", file=sys.stderr)
        return None

def main():
    import sys
    
    rclpy.init()
    node = Node('activate_controller')
    
    # Client per list_controllers (verifica stato)
    client_list = node.create_client(ListControllers, '/controller_manager/list_controllers')
    
    # Client per switch_controller
    client_switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
    
    # PRIMA: Verifica stato corrente
    print("⏳ Verifica stato corrente controller...", file=sys.stderr)
    current_status = get_controller_status(node, client_list)
    
    if current_status is None:
        print("WARNING: Impossibile verificare stato controller, procedo comunque...", file=sys.stderr)
    else:
        print(f"✅ Stato corrente: {current_status}", file=sys.stderr)
        
        # Se forward_velocity_controller è già attivo, ritorna successo
        if 'forward_velocity_controller' in current_status:
            if current_status['forward_velocity_controller'] == 'active':
                print("OK: Controller forward_velocity_controller già attivo - nessuna azione necessaria")
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
        
        # Se scaled_joint_trajectory_controller non esiste o non è attivo, non provare a deattivarlo
        deactivate_list = []
        if 'scaled_joint_trajectory_controller' in current_status:
            if current_status['scaled_joint_trajectory_controller'] == 'active':
                deactivate_list = ['scaled_joint_trajectory_controller']
            else:
                print("INFO: scaled_joint_trajectory_controller non attivo - salto deattivazione", file=sys.stderr)
        else:
            print("INFO: scaled_joint_trajectory_controller non esiste - salto deattivazione", file=sys.stderr)
    
    # Verifica servizio switch_controller
    if not client_switch.wait_for_service(timeout_sec=5.0):
        print("ERROR: Servizio switch_controller non disponibile!", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    
    # Prepara richiesta switch
    request = SwitchController.Request()
    request.activate_controllers = ['forward_velocity_controller']
    request.deactivate_controllers = deactivate_list
    request.strictness = SwitchController.Request.BEST_EFFORT  # 0 = BEST_EFFORT (permette switch parziali)
    request.activate_asap = True
    # Timeout deve essere un messaggio builtin_interfaces/Duration, non rclpy.duration.Duration
    from builtin_interfaces.msg import Duration
    request.timeout = Duration()
    request.timeout.sec = 5
    request.timeout.nanosec = 0
    
    print(f"🔄 Switch controller: activate={request.activate_controllers}, deactivate={request.deactivate_controllers}, strictness=BEST_EFFORT", file=sys.stderr)
    
    future = client_switch.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    if future.result() is not None:
        response = future.result()
        if response.ok:
            print("OK: Controller forward_velocity_controller attivato con successo")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        else:
            print(f"WARNING: Switch controller fallito: {response}", file=sys.stderr)
    else:
        print("WARNING: Nessuna risposta dal servizio switch_controller", file=sys.stderr)
    
    # Verifica finale stato
    print("⏳ Verifica finale stato controller...", file=sys.stderr)
    final_status = get_controller_status(node, client_list)
    
    if final_status and 'forward_velocity_controller' in final_status:
        if final_status['forward_velocity_controller'] == 'active':
            print("OK: Controller forward_velocity_controller risulta attivo (verifica finale)")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
    
    # Se arriviamo qui, il controller non è attivo
    print("ERROR: Impossibile attivare controller forward_velocity_controller", file=sys.stderr)
    print("   Verifica che:", file=sys.stderr)
    print("   1. Il driver ROS2 sia avviato: pgrep -f ur_ros2_control_node", file=sys.stderr)
    print("   2. Il controller_manager sia attivo: ros2 service list | grep controller_manager", file=sys.stderr)
    print("   3. Il robot sia in Remote Control sul Teach Pendant", file=sys.stderr)
    
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

if __name__ == '__main__':
    main()

