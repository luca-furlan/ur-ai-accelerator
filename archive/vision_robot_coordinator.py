#!/usr/bin/env python3
"""
Vision Robot Coordinator - Nodo coordinatore principale
Gestisce pipeline completa: Camera → YOLO Detection → MoveIt Planning → Robot Control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import sys
import os
from typing import Dict, Optional, List
from enum import Enum

# Importa controller robot esistente
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from remote_ur_control.remote_ur_controller import RemoteURController, MoveParameters
    UR_CONTROLLER_AVAILABLE = True
except ImportError:
    UR_CONTROLLER_AVAILABLE = False
    print("⚠️ RemoteURController non disponibile")


class SystemState(Enum):
    """Stati del sistema"""
    IDLE = "idle"
    DETECTING = "detecting"
    PLANNING = "planning"
    MOVING = "moving"
    ERROR = "error"
    PAUSED = "paused"


class VisionRobotCoordinator(Node):
    """
    Coordinatore centrale che gestisce l'intero sistema:
    1. Monitora detections da vision node
    2. Riceve comandi da web interface
    3. Coordina movimenti robot con sicurezza
    4. Gestisce stato sistema
    """
    
    def __init__(self):
        super().__init__('vision_robot_coordinator')
        
        # Parametri
        self.declare_parameter('robot_ip', '192.168.10.194')
        self.declare_parameter('enable_robot_control', True)
        self.declare_parameter('safe_mode', True)
        self.declare_parameter('auto_pick', False)
        
        self.robot_ip = self.get_parameter('robot_ip').value
        self.enable_control = self.get_parameter('enable_robot_control').value
        self.safe_mode = self.get_parameter('safe_mode').value
        self.auto_pick = self.get_parameter('auto_pick').value
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("VISION ROBOT COORDINATOR")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Robot IP: {self.robot_ip}")
        self.get_logger().info(f"  Robot Control: {self.enable_control}")
        self.get_logger().info(f"  Safe Mode: {self.safe_mode}")
        self.get_logger().info(f"  Auto Pick: {self.auto_pick}")
        
        # Stato sistema
        self.state = SystemState.IDLE
        self.latest_detections: List[Dict] = []
        self.selected_target: Optional[Dict] = None
        self.robot_controller: Optional[RemoteURController] = None
        
        # Inizializza robot controller
        if self.enable_control and UR_CONTROLLER_AVAILABLE:
            try:
                self.get_logger().info("Connessione a robot...")
                self.robot_controller = RemoteURController(self.robot_ip)
                self.get_logger().info("✅ Robot controller connesso")
            except Exception as e:
                self.get_logger().error(f"❌ Errore connessione robot: {e}")
                self.robot_controller = None
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            String,
            '/vision/detections_3d',
            self.detections_callback,
            10
        )
        
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/selected_target',
            self.target_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/vision/coordinator_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(
            String,
            '/vision/system_status',
            10
        )
        
        self.response_pub = self.create_publisher(
            String,
            '/vision/coordinator_response',
            10
        )
        
        # Timer per status update
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Statistiche
        self.total_detections = 0
        self.total_movements = 0
        self.successful_movements = 0
        
        self.get_logger().info("✅ Vision Robot Coordinator ONLINE")
        self.get_logger().info("=" * 60)
    
    def detections_callback(self, msg: String):
        """Riceve detections 3D"""
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            if detections:
                self.latest_detections = detections
                self.total_detections += len(detections)
                
                if self.state == SystemState.IDLE:
                    self.state = SystemState.DETECTING
                
                # Log detections interessanti
                classes = [d['class'] for d in detections]
                self.get_logger().info(
                    f"📷 Detections: {len(detections)} objects - {', '.join(set(classes))}",
                    throttle_duration_sec=2.0
                )
        
        except Exception as e:
            self.get_logger().error(f"Errore processing detections: {e}")
    
    def target_callback(self, msg: PoseStamped):
        """Riceve target selezionato da MoveIt controller"""
        # Salva target
        self.selected_target = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        
        self.get_logger().info(
            f"🎯 Target selezionato: ({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})"
        )
    
    def command_callback(self, msg: String):
        """
        Riceve comandi da web interface o altri nodi
        
        Comandi supportati:
        - pick_target: inizia sequenza pick del target selezionato
        - stop: ferma robot
        - pause: pausa operazioni
        - resume: riprendi operazioni
        - reset: reset stato
        """
        try:
            cmd_data = json.loads(msg.data)
            command = cmd_data.get('command')
            
            self.get_logger().info(f"📥 Comando ricevuto: {command}")
            
            if command == 'pick_target':
                self.execute_pick_sequence(cmd_data)
            
            elif command == 'stop':
                self.execute_stop()
            
            elif command == 'pause':
                self.state = SystemState.PAUSED
                self.send_response({'status': 'paused'})
            
            elif command == 'resume':
                self.state = SystemState.IDLE
                self.send_response({'status': 'resumed'})
            
            elif command == 'reset':
                self.reset_system()
            
            elif command == 'move_to_position':
                # Movimento diretto a posizione specifica
                position = cmd_data.get('position')
                self.move_to_position(position)
            
            else:
                self.get_logger().warn(f"Comando sconosciuto: {command}")
                self.send_response({'status': 'error', 'message': 'Unknown command'})
        
        except Exception as e:
            self.get_logger().error(f"Errore processing comando: {e}")
            self.send_response({'status': 'error', 'message': str(e)})
    
    def execute_pick_sequence(self, cmd_data: Dict):
        """
        Esegue sequenza completa di pick:
        1. Move to approach pose (sopra oggetto)
        2. Move down to grasp
        3. Close gripper
        4. Move up
        5. Move to place location
        """
        if not self.robot_controller:
            self.get_logger().error("Robot controller non disponibile")
            self.send_response({'status': 'error', 'message': 'Robot not connected'})
            return
        
        if self.state == SystemState.MOVING:
            self.get_logger().warn("Robot già in movimento")
            self.send_response({'status': 'error', 'message': 'Robot already moving'})
            return
        
        target = cmd_data.get('target') or self.selected_target
        if not target:
            self.get_logger().error("Nessun target selezionato")
            self.send_response({'status': 'error', 'message': 'No target selected'})
            return
        
        self.state = SystemState.MOVING
        self.total_movements += 1
        
        try:
            position = target['position']
            
            self.get_logger().info(f"🤖 Inizio sequenza pick verso ({position['x']:.2f}, {position['y']:.2f}, {position['z']:.2f})")
            
            # Converti coordinate camera → base robot (se necessario)
            # TODO: implementare trasformazione con TF2
            
            # Per ora: movimento semplice verso target
            # In produzione: sequenza completa con approach, grasp, retreat
            
            # MOVIMENTO LINEARE verso target
            success = self.robot_controller.movel_relative(
                position=[position['x'], position['y'], position['z'], 0, 0, 0],
                speed=0.1,
                acceleration=0.05
            )
            
            if success:
                self.successful_movements += 1
                self.get_logger().info("✅ Movimento completato con successo")
                self.send_response({
                    'status': 'success',
                    'message': 'Pick sequence completed',
                    'target': position
                })
            else:
                self.get_logger().error("❌ Movimento fallito")
                self.send_response({
                    'status': 'error',
                    'message': 'Movement failed'
                })
        
        except Exception as e:
            self.get_logger().error(f"❌ Errore durante pick: {e}")
            self.send_response({
                'status': 'error',
                'message': f'Pick failed: {str(e)}'
            })
        
        finally:
            self.state = SystemState.IDLE
    
    def move_to_position(self, position: Dict):
        """Movimento semplice verso posizione"""
        if not self.robot_controller:
            self.get_logger().error("Robot controller non disponibile")
            return
        
        try:
            self.state = SystemState.MOVING
            
            # Movimento verso posizione
            success = self.robot_controller.movel_relative(
                position=[
                    position.get('x', 0),
                    position.get('y', 0),
                    position.get('z', 0),
                    position.get('rx', 0),
                    position.get('ry', 0),
                    position.get('rz', 0)
                ],
                speed=position.get('speed', 0.1),
                acceleration=position.get('acceleration', 0.05)
            )
            
            if success:
                self.get_logger().info("✅ Movimento completato")
                self.send_response({'status': 'success'})
            else:
                self.get_logger().error("❌ Movimento fallito")
                self.send_response({'status': 'error'})
        
        except Exception as e:
            self.get_logger().error(f"Errore movimento: {e}")
            self.send_response({'status': 'error', 'message': str(e)})
        
        finally:
            self.state = SystemState.IDLE
    
    def execute_stop(self):
        """Ferma robot"""
        if self.robot_controller:
            try:
                self.robot_controller.stop()
                self.get_logger().info("🛑 Robot fermato")
                self.state = SystemState.IDLE
                self.send_response({'status': 'stopped'})
            except Exception as e:
                self.get_logger().error(f"Errore stop: {e}")
        else:
            self.get_logger().warn("Robot controller non disponibile per stop")
    
    def reset_system(self):
        """Reset stato sistema"""
        self.state = SystemState.IDLE
        self.selected_target = None
        self.latest_detections = []
        self.get_logger().info("🔄 Sistema resettato")
        self.send_response({'status': 'reset'})
    
    def publish_status(self):
        """Pubblica stato sistema periodicamente"""
        status = {
            'state': self.state.value,
            'robot_connected': self.robot_controller is not None,
            'detections_count': len(self.latest_detections),
            'has_target': self.selected_target is not None,
            'statistics': {
                'total_detections': self.total_detections,
                'total_movements': self.total_movements,
                'successful_movements': self.successful_movements
            },
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def send_response(self, response: Dict):
        """Invia risposta a comando"""
        msg = String()
        msg.data = json.dumps(response)
        self.response_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = VisionRobotCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutdown Vision Robot Coordinator")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




