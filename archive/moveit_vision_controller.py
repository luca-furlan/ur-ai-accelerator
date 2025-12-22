#!/usr/bin/env python3
"""
ROS2 Node: MoveIt2 + Vision Integration Controller
Riceve detections 3D e coordina movimenti robot con MoveIt2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import json
import numpy as np
from typing import Optional, Dict, List
import threading

try:
    from moveit_msgs.srv import GetPositionIK, GetPositionFK
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class MoveItVisionController(Node):
    """
    Nodo che integra Vision + MoveIt2:
    1. Riceve detections 3D da vision node
    2. Seleziona target basato su priorità
    3. Pianifica movimento con MoveIt2
    4. Esegue movimento sicuro verso target
    """
    
    def __init__(self):
        super().__init__('moveit_vision_controller')
        
        # Parametri
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('target_classes', ['bottle', 'cup', 'person', 'cell phone'])
        self.declare_parameter('approach_offset', 0.15)  # Offset sopra oggetto (m)
        self.declare_parameter('auto_mode', False)  # Se True, muove automaticamente
        self.declare_parameter('safety_distance', 0.1)  # Distanza sicurezza (m)
        
        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.ref_frame = self.get_parameter('reference_frame').value
        self.target_classes = self.get_parameter('target_classes').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.auto_mode = self.get_parameter('auto_mode').value
        self.safety_dist = self.get_parameter('safety_distance').value
        
        self.get_logger().info("Inizializzazione MoveIt Vision Controller...")
        self.get_logger().info(f"  Planning Group: {self.planning_group}")
        self.get_logger().info(f"  End Effector: {self.ee_link}")
        self.get_logger().info(f"  Target Classes: {self.target_classes}")
        self.get_logger().info(f"  Auto Mode: {self.auto_mode}")
        
        # Stato
        self.latest_detections: List[Dict] = []
        self.current_target: Optional[Dict] = None
        self.is_moving = False
        self.movement_lock = threading.Lock()
        
        # Subscriber detections 3D
        self.detections_sub = self.create_subscription(
            String,
            '/vision/detections_3d',
            self.detections_callback,
            10
        )
        
        # Publisher per comandi robot (diretto)
        self.robot_command_pub = self.create_publisher(
            String,
            '/robot/vision_command',
            10
        )
        
        # Publisher per target selezionato
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/vision/selected_target',
            10
        )
        
        # Publisher per status
        self.status_pub = self.create_publisher(
            String,
            '/vision/controller_status',
            10
        )
        
        # Service per comando manuale
        # TODO: aggiungere service per "pick this object"
        
        # Timer per processing
        self.timer = self.create_timer(0.5, self.process_detections)
        
        self.get_logger().info("✅ MoveIt Vision Controller avviato")
        if not self.auto_mode:
            self.get_logger().info("   MANUAL MODE: usa /vision/pick_object service")
        else:
            self.get_logger().info("   AUTO MODE: movimento automatico abilitato")
    
    def detections_callback(self, msg: String):
        """Riceve detections 3D da vision node"""
        try:
            data = json.loads(msg.data)
            self.latest_detections = data.get('detections', [])
            
            # Log detections interessanti
            interesting = [d for d in self.latest_detections 
                          if d['class'] in self.target_classes]
            
            if interesting:
                self.get_logger().info(
                    f"Detections: {len(self.latest_detections)} totali, "
                    f"{len(interesting)} target interessanti"
                )
        
        except Exception as e:
            self.get_logger().error(f"Errore parsing detections: {e}")
    
    def process_detections(self):
        """Processa detections e decide target"""
        if not self.latest_detections:
            return
        
        # Filtra per classi interessanti
        targets = [d for d in self.latest_detections 
                  if d['class'] in self.target_classes]
        
        if not targets:
            return
        
        # Seleziona target (più vicino al centro immagine, oppure più vicino al robot)
        best_target = self.select_best_target(targets)
        
        if best_target != self.current_target:
            self.current_target = best_target
            
            # Pubblica target selezionato
            self.publish_target(best_target)
            
            # Se auto mode, inizia movimento
            if self.auto_mode and not self.is_moving:
                self.get_logger().info(
                    f"🎯 Nuovo target: {best_target['class']} "
                    f"@ ({best_target['position_3d']['x']:.2f}, "
                    f"{best_target['position_3d']['y']:.2f}, "
                    f"{best_target['position_3d']['z']:.2f})"
                )
                self.move_to_target(best_target)
    
    def select_best_target(self, targets: List[Dict]) -> Dict:
        """
        Seleziona il miglior target dalla lista
        Criteri: distanza dal robot, confidence, priorità classe
        """
        # Score basato su:
        # - Distanza dal robot (più vicino = meglio, ma non troppo vicino)
        # - Confidence della detection
        # - Priorità classe (opzionale)
        
        best_target = None
        best_score = -float('inf')
        
        for target in targets:
            pos = target['position_3d']
            distance = np.sqrt(pos['x']**2 + pos['y']**2 + pos['z']**2)
            confidence = target['confidence']
            
            # Evita oggetti troppo vicini o troppo lontani
            if distance < self.safety_dist or distance > 2.0:
                continue
            
            # Score: privilegia confidence alta e distanza media
            score = confidence * 2.0 - abs(distance - 0.5) * 0.5
            
            if score > best_score:
                best_score = score
                best_target = target
        
        return best_target if best_target else targets[0]
    
    def publish_target(self, target: Dict):
        """Pubblica target selezionato come PoseStamped"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.ref_frame
        
        # Posizione: aggiungi approach offset sull'asse Z
        pos = target['position_3d']
        pose_msg.pose.position.x = pos['x']
        pose_msg.pose.position.y = pos['y']
        pose_msg.pose.position.z = pos['z'] + self.approach_offset
        
        # Orientazione: guarda verso il basso
        # Quaternion per guardare -Z (tool verso basso)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 1.0  # 180° rotation around Y
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0
        
        self.target_pub.publish(pose_msg)
    
    def move_to_target(self, target: Dict):
        """
        Muove robot verso target usando comando diretto
        (MoveIt2 integration può essere aggiunta dopo)
        """
        with self.movement_lock:
            if self.is_moving:
                self.get_logger().warn("Movimento già in corso, skip")
                return
            
            self.is_moving = True
        
        try:
            pos = target['position_3d']
            
            # Crea comando JSON per robot
            command = {
                'command': 'move_to_detected_object',
                'target': {
                    'class': target['class'],
                    'confidence': target['confidence'],
                    'position': {
                        'x': pos['x'],
                        'y': pos['y'],
                        'z': pos['z'] + self.approach_offset
                    }
                },
                'mode': 'linear',  # o 'joint'
                'speed': 0.1,
                'acceleration': 0.05
            }
            
            # Pubblica comando
            cmd_msg = String()
            cmd_msg.data = json.dumps(command)
            self.robot_command_pub.publish(cmd_msg)
            
            self.get_logger().info(f"✅ Comando movimento inviato verso {target['class']}")
            
            # Pubblica status
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'moving',
                'target': target['class'],
                'position': pos
            })
            self.status_pub.publish(status_msg)
        
        except Exception as e:
            self.get_logger().error(f"Errore movimento: {e}")
        
        finally:
            # Reset flag dopo timeout (il movimento vero verrà confermato dal robot)
            self.create_timer(5.0, lambda: self.reset_moving_flag(), clock=self.get_clock())
    
    def reset_moving_flag(self):
        """Reset flag movimento"""
        with self.movement_lock:
            self.is_moving = False
    
    def transform_camera_to_base(self, point_camera: Dict) -> Dict:
        """
        Trasforma punto da frame camera a frame base robot
        TODO: implementare con tf2_ros per trasformazione corretta
        
        Per ora: assume camera montata sul robot o calibrata
        """
        # PLACEHOLDER: assumiamo frame camera coincidente con base
        # In produzione: usare tf2_ros.TransformListener
        
        return {
            'x': point_camera['x'],
            'y': point_camera['y'],
            'z': point_camera['z']
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItVisionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutdown MoveIt Vision Controller")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




