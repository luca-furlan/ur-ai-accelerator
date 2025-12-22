#!/usr/bin/env python3
"""
ROS2 node for UR robot control via RTDE.
Subscribes to commands and publishes joint states.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sys
import os

# Add parent directory to path to import rtde_teleop
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../../projects/ur-ai-accelerator-remote/remote_ur_control'))

try:
    from rtde_teleop import RTDETeleop, RTDEConfig
except ImportError:
    try:
        from remote_ur_control.rtde_teleop import RTDETeleop, RTDEConfig
    except ImportError:
        print('ERROR: Cannot import rtde_teleop. Install ur-rtde: pip install ur-rtde')
        sys.exit(1)


class URControllerNode(Node):
    def __init__(self):
        super().__init__('ur_controller_node')
        
        # Parameters
        self.declare_parameter('robot_ip', '192.168.10.194')
        self.declare_parameter('frequency', 125.0)
        self.declare_parameter('velocity', 0.2)
        self.declare_parameter('acceleration', 0.7)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        velocity = self.get_parameter('velocity').get_parameter_value().double_value
        acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'Connecting to UR robot at {robot_ip}...')
        
        # Initialize RTDE
        config = RTDEConfig(
            robot_ip=robot_ip,
            frequency=frequency,
            velocity=velocity,
            acceleration=acceleration
        )
        self.teleop = RTDETeleop(config)
        self.get_logger().info('✅ Connected to UR robot')
        
        # Subscribers
        self.create_subscription(
            Float64MultiArray,
            '/ur_controller/movej',
            self.movej_callback,
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            '/ur_controller/speedj',
            self.speedj_callback,
            10
        )
        
        self.create_subscription(
            Twist,
            '/ur_controller/twist',
            self.twist_callback,
            10
        )
        
        self.create_subscription(
            Empty,
            '/ur_controller/stop',
            self.stop_callback,
            10
        )
        
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/ur_controller/joint_states',
            10
        )
        
        # Timer for publishing joint states
        self.create_timer(1.0 / publish_rate, self.publish_joint_states)
        
        self.get_logger().info('UR Controller Node ready!')
        self.get_logger().info('Topics:')
        self.get_logger().info('  Sub: /ur_controller/movej (Float64MultiArray)')
        self.get_logger().info('  Sub: /ur_controller/speedj (Float64MultiArray)')
        self.get_logger().info('  Sub: /ur_controller/twist (Twist)')
        self.get_logger().info('  Sub: /ur_controller/stop (Empty)')
        self.get_logger().info('  Pub: /ur_controller/joint_states (JointState)')
    
    def movej_callback(self, msg):
        if len(msg.data) != 6:
            self.get_logger().error(f'movej expects 6 joints, got {len(msg.data)}')
            return
        try:
            self.teleop.movej(list(msg.data), wait=False)
            self.get_logger().info(f'movej: {[f"{x:.3f}" for x in msg.data]}')
        except Exception as e:
            self.get_logger().error(f'movej failed: {e}')
    
    def speedj_callback(self, msg):
        if len(msg.data) != 6:
            self.get_logger().error(f'speedj expects 6 joints, got {len(msg.data)}')
            return
        try:
            self.teleop.speedj(list(msg.data), duration=0.5)
            self.get_logger().debug(f'speedj: {[f"{x:.3f}" for x in msg.data]}')
        except Exception as e:
            self.get_logger().error(f'speedj failed: {e}')
    
    def twist_callback(self, msg):
        # Convert Twist to speedj (simple mapping: linear -> joints 0,1,2; angular -> joints 3,4,5)
        joint_speeds = [
            msg.linear.x * 0.5,   # Joint 0
            msg.linear.y * 0.5,   # Joint 1
            msg.linear.z * 0.5,   # Joint 2
            msg.angular.x * 0.5,  # Joint 3
            msg.angular.y * 0.5,  # Joint 4
            msg.angular.z * 0.5,  # Joint 5
        ]
        try:
            self.teleop.speedj(joint_speeds, duration=0.3)
        except Exception as e:
            self.get_logger().error(f'twist failed: {e}')
    
    def stop_callback(self, msg):
        try:
            self.teleop.halt()
            self.get_logger().warn('🛑 STOP command received')
        except Exception as e:
            self.get_logger().error(f'stop failed: {e}')
    
    def publish_joint_states(self):
        try:
            joints = self.teleop.get_joints()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'joint_{i+1}' for i in range(6)]
            msg.position = list(joints)
            msg.velocity = [0.0] * 6  # RTDE doesn't provide velocity directly
            msg.effort = [0.0] * 6
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish joint states: {e}')
    
    def destroy_node(self):
        self.get_logger().info('Shutting down UR Controller Node...')
        try:
            self.teleop.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = URControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()






















