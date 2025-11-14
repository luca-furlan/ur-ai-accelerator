#!/usr/bin/env python3
"""
ROS2 bridge for web interface.
Provides publishers for UR robot commands via ROS2 topics.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from geometry_msgs.msg import Twist
import threading


class ROS2Bridge:
    """Singleton ROS2 bridge for web interface."""
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self._initialized = True
        self._node = None
        self._publishers = {}
        self._ros_thread = None
        self._ros_initialized = False
        
    def _init_ros(self):
        """Initialize ROS2 in a separate thread."""
        if self._ros_initialized:
            return
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self._node = Node('web_interface_bridge')
            
            # Create publishers
            self._publishers['movej'] = self._node.create_publisher(
                Float64MultiArray,
                '/ur_controller/movej',
                10
            )
            
            self._publishers['speedj'] = self._node.create_publisher(
                Float64MultiArray,
                '/ur_controller/speedj',
                10
            )
            
            self._publishers['twist'] = self._node.create_publisher(
                Twist,
                '/ur_controller/twist',
                10
            )
            
            self._publishers['stop'] = self._node.create_publisher(
                Empty,
                '/ur_controller/stop',
                10
            )
            
            self._ros_initialized = True
            print("✅ ROS2 bridge initialized")
            
            # Spin in background
            def spin_ros():
                try:
                    rclpy.spin(self._node)
                except:
                    pass
            
            self._ros_thread = threading.Thread(target=spin_ros, daemon=True)
            self._ros_thread.start()
            
        except Exception as e:
            print(f"⚠️ Failed to initialize ROS2: {e}")
            print("⚠️ Falling back to direct socket connection")
            self._ros_initialized = False
    
    def ensure_ros(self):
        """Ensure ROS2 is initialized."""
        if not self._ros_initialized:
            self._init_ros()
        return self._ros_initialized
    
    def publish_movej(self, joints):
        """Publish movej command."""
        if not self.ensure_ros():
            return False
        
        msg = Float64MultiArray()
        msg.data = [float(j) for j in joints]
        self._publishers['movej'].publish(msg)
        return True
    
    def publish_speedj(self, speeds):
        """Publish speedj command."""
        if not self.ensure_ros():
            return False
        
        msg = Float64MultiArray()
        msg.data = [float(s) for s in speeds]
        self._publishers['speedj'].publish(msg)
        return True
    
    def publish_twist(self, linear, angular):
        """Publish Twist command."""
        if not self.ensure_ros():
            return False
        
        msg = Twist()
        msg.linear.x = float(linear[0])
        msg.linear.y = float(linear[1])
        msg.linear.z = float(linear[2])
        msg.angular.x = float(angular[0])
        msg.angular.y = float(angular[1])
        msg.angular.z = float(angular[2])
        self._publishers['twist'].publish(msg)
        return True
    
    def publish_stop(self):
        """Publish stop command."""
        if not self.ensure_ros():
            return False
        
        msg = Empty()
        self._publishers['stop'].publish(msg)
        return True
    
    def shutdown(self):
        """Shutdown ROS2."""
        if self._node and rclpy.ok():
            try:
                self._node.destroy_node()
                rclpy.shutdown()
            except:
                pass

















