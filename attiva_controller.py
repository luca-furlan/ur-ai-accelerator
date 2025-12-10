#!/usr/bin/env python3
"""Script Python per attivare forward_velocity_controller."""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController

def main():
    rclpy.init()
    node = Node('activate_controller')
    
    client = node.create_client(SwitchController, '/controller_manager/switch_controllers')
    
    if not client.wait_for_service(timeout_sec=5.0):
        print("❌ Servizio switch_controllers non disponibile!")
        return
    
    request = SwitchController.Request()
    request.activate_controllers = ['forward_velocity_controller']
    request.deactivate_controllers = ['scaled_joint_trajectory_controller']
    request.strictness = SwitchController.Request.STRICT
    
    print("🔄 Attivazione forward_velocity_controller...")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        response = future.result()
        if response.ok:
            print("✅ Controller attivato con successo!")
        else:
            print(f"❌ Errore: {response}")
    else:
        print("❌ Nessuna risposta dal servizio")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

