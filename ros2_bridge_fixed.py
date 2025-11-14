#!/usr/bin/env python3
"""
ROS2 bridge for web interface.
Provides publishers for UR robot commands via ROS2 topics.
"""
import os
import platform
import sys
from datetime import datetime

# CRITICAL: Set LD_LIBRARY_PATH BEFORE importing rclpy
# This must happen before any Python imports that load shared libraries
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
    import subprocess
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
                    if key in ['LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION']:
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
        print(f'⚠️ Warning: Could not source ROS2 setup: {e}')
        # Fallback: set common paths anyway
        ros_lib = '/opt/ros/humble/lib'
        current_ld = os.environ.get('LD_LIBRARY_PATH', '')
        if ros_lib not in current_ld:
            os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib

# Setup ROS2 environment BEFORE importing rclpy
setup_ros2_environment()

# Now try to import rclpy
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Empty
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
    print(f'✅ ROS2 available - LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "not set")[:100]}...')
except ImportError as e:
    ROS2_AVAILABLE = False
    Node = None
    print(f'⚠️ ROS2 not available: {e}')
    print(f'   LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "not set")}')
    print(f'   PYTHONPATH={os.environ.get("PYTHONPATH", "not set")[:100]}...')

import threading
import time
import subprocess


class ROS2Bridge:
    """Singleton ROS2 bridge for web interface.
    
    Implements continuous publishing at 125Hz (8ms) for smooth control,
    following Universal Robots ROS2 driver best practices.
    """
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
        self._publish_thread = None
        self._ros_initialized = False
        self._target_speeds = [0.0] * 6   # Desired velocities from web UI
        self._current_speeds = [0.0] * 6  # Smoothed velocities actually published
        self._speed_lock = threading.Lock()
        self._publish_rate = 125.0  # Hz - standard UR RTDE frequency
        self._smoothing_factor = 0.15  # Exponential smoothing: 0.0 = no smoothing, 1.0 = no change
        # Lower value = faster response but smoother blending (0.1-0.2 is good for fluid motion)
        self._running = False
        self._last_command_time = None
        self._last_publish_time = None
        self._last_error = None
        self._last_command_payload = [0.0] * 6
        
    def _init_ros(self):
        """Initialize ROS2 in a separate thread."""
        if self._ros_initialized:
            return
        
        if not ROS2_AVAILABLE:
            print('⚠️ ROS2 not available (rclpy not found). Web interface will work but ROS2 commands will fail.')
            return
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self._node = Node('web_interface_bridge')
            
            # Create publishers - usa topic reali del driver UR
            # Per movej: usa joint_trajectory_controller
            try:
                from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
                self._publishers['movej'] = self._node.create_publisher(
                    JointTrajectory,
                    '/scaled_joint_trajectory_controller/joint_trajectory',
                    10
                )
                self._trajectory_msg_type = JointTrajectory
                self._trajectory_point_type = JointTrajectoryPoint
            except ImportError:
                self._publishers['movej'] = None
                self._trajectory_msg_type = None
                self._trajectory_point_type = None
            
            # Per speedj: usa forward_velocity_controller (controllo velocità diretto)
            # Questo è il controller principale per controllo fluido in ROS2
            self._publishers['speedj'] = self._node.create_publisher(
                Float64MultiArray,
                '/forward_velocity_controller/commands',
                10
            )
            
            # Twist per controllo cartesiano - il driver UR non ha un topic twist diretto
            # Per ora usiamo solo joint control via forward_velocity_controller
            self._publishers['twist'] = None
            self._publishers['servo_twist'] = None
            
            # Stop: usa lo stesso publisher di speedj (invia velocità zero)
            self._publishers['stop'] = self._publishers['speedj']
            
            self._ros_initialized = True
            print('✅ ROS2 bridge initialized')
            
            # Spin in background
            def spin_ros():
                try:
                    rclpy.spin(self._node)
                except:
                    pass
            
            self._ros_thread = threading.Thread(target=spin_ros, daemon=True)
            self._ros_thread.start()
            
            # Start continuous publishing loop at 125Hz
            self._start_publish_loop()
            
        except Exception as e:
            print(f'⚠️ Failed to initialize ROS2: {e}')
            import traceback
            traceback.print_exc()
            self._ros_initialized = False
    
    def ensure_ros(self):
        """Ensure ROS2 is initialized."""
        if not self._ros_initialized:
            self._init_ros()
        return self._ros_initialized
    
    def _start_publish_loop(self):
        """Start continuous publishing loop at 125Hz for smooth control."""
        if self._publish_thread and self._publish_thread.is_alive():
            print('⚠️ Publish loop already running')
            return
        
        if not self._ros_initialized:
            print('⚠️ Cannot start publish loop: ROS2 not initialized')
            return
        
        if not self._publishers.get('speedj'):
            print('⚠️ Cannot start publish loop: speedj publisher not created')
            return
        
        self._running = True
        print(f'🔄 Starting publish loop at {self._publish_rate}Hz...')
        
        def publish_loop():
            """Continuous publishing loop at 125Hz (8ms interval)."""
            interval = 1.0 / self._publish_rate  # 0.008 seconds = 8ms
            publish_count = 0
            print(f'🔄 Publish loop started (target: {self._publish_rate}Hz)')
            
            while self._running and self._ros_initialized:
                try:
                    # Get targets and currents (thread-safe) and apply exponential smoothing
                    with self._speed_lock:
                        targets = list(self._target_speeds)
                        currents = list(self._current_speeds)
                    
                    # Apply exponential smoothing filter for fluid blending (no "tac tac")
                    # Formula: new = current + alpha * (target - current)
                    # Lower alpha = smoother but slower response
                    # Higher alpha = faster but potentially jerky
                    alpha = self._smoothing_factor
                    updated = []
                    for target, current in zip(targets, currents):
                        # Exponential smoothing: smooth transition without steps
                        new_val = current + alpha * (target - current)
                        # If very close to target, snap to it to avoid floating point drift
                        if abs(target - new_val) < 0.001:
                            new_val = target
                        updated.append(new_val)
                    
                    with self._speed_lock:
                        self._current_speeds = updated
                    
                    speeds = list(updated)

                    # Publish current speeds
                    publisher = self._publishers.get('speedj')
                    if publisher:
                        msg = Float64MultiArray()
                        msg.data = speeds
                        publisher.publish(msg)
                        publish_count += 1
                        self._last_publish_time = time.time()
                        if publish_count % 125 == 0:  # Log ogni secondo
                            print(f'📤 Published {publish_count} messages (current speeds: {[f"{s:.3f}" for s in speeds]})')
                    else:
                        print('⚠️ Publisher not available')
                        break
                    
                    # Sleep for exactly 8ms (125Hz)
                    time.sleep(interval)
                except Exception as e:
                    print(f'❌ Error in publish loop: {e}')
                    self._last_error = str(e)
                    import traceback
                    traceback.print_exc()
                    time.sleep(interval)
            
            print(f'🛑 Publish loop stopped (published {publish_count} messages total)')
        
        self._publish_thread = threading.Thread(target=publish_loop, daemon=True)
        self._publish_thread.start()
        print(f'✅ Started continuous publishing loop at {self._publish_rate}Hz')
    
    def publish_movej(self, joints):
        """Publish movej command usando JointTrajectory."""
        if not self.ensure_ros():
            return False
        
        try:
            if self._publishers.get('movej') and self._trajectory_msg_type:
                msg = self._trajectory_msg_type()
                point = self._trajectory_point_type()
                point.positions = [float(j) for j in joints]
                point.time_from_start.sec = 1
                point.time_from_start.nanosec = 0
                msg.points = [point]
                # Nomi joint standard UR
                msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                                  'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                self._publishers['movej'].publish(msg)
                return True
            else:
                # Fallback: usa speedj con velocità zero dopo movimento
                return False
        except Exception as e:
            print(f'Error publishing movej: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def publish_speedj(self, speeds):
        """Update target speeds (will be published continuously at 125Hz).
        
        This method updates the target velocities that are published
        continuously by the background thread. This ensures smooth,
        high-frequency control as recommended by Universal Robots.
        """
        if not self.ensure_ros():
            return False
        
        try:
            # Update target speeds (thread-safe)
            with self._speed_lock:
                self._target_speeds = [float(s) for s in speeds]
                self._last_command_payload = list(self._target_speeds)
            self._last_command_time = time.time()
            return True
        except Exception as e:
            print(f'Error updating speeds: {e}')
            self._last_error = str(e)
            return False
    
    def publish_twist(self, linear, angular, use_servo_node=True):
        """Publish Twist command. 
        Nota: Il driver UR standard non supporta controllo cartesiano diretto via ROS2.
        Per controllo fluido, usa publish_speedj con velocità joint calcolate.
        """
        if not self.ensure_ros():
            return False
        
        # Il driver UR non ha un topic twist diretto
        # Per controllo cartesiano, il web interface converte già twist in joint velocities
        # Quindi restituiamo False per far usare speedj al web interface
        # Questo è il comportamento corretto: il web interface invia già speeds come joint velocities
        return False
    
    def publish_stop(self):
        """Stop command - sets all velocities to zero."""
        if not self.ensure_ros():
            return False
        
        try:
            # Set all speeds to zero (will be published continuously)
            with self._speed_lock:
                self._target_speeds = [0.0] * 6
                self._current_speeds = [0.0] * 6
                self._last_command_payload = [0.0] * 6
            self._last_command_time = time.time()
            return True
        except Exception as e:
            print(f'Error stopping: {e}')
            self._last_error = str(e)
            return False
    
    def shutdown(self):
        """Shutdown ROS2 and stop publishing loop."""
        self._running = False
        
        if self._publish_thread:
            self._publish_thread.join(timeout=1.0)
        
        if self._node and ROS2_AVAILABLE and rclpy.ok():
            try:
                self._node.destroy_node()
                rclpy.shutdown()
            except:
                pass

    def get_status(self):
        """Return a dictionary with diagnostics for the web interface."""
        now = time.time()

        def _ago(ts):
            if not ts:
                return None
            return max(0.0, now - ts)

        def _iso(ts):
            if not ts:
                return None
            return datetime.fromtimestamp(ts).isoformat(timespec='seconds')

        publishers = {name: pub is not None for name, pub in self._publishers.items()}

        return {
            'ros2_available': ROS2_AVAILABLE,
            'ros_initialized': self._ros_initialized,
            'rclpy_ok': bool(rclpy.ok()) if ROS2_AVAILABLE else False,
            'publish_loop_running': bool(self._publish_thread and self._publish_thread.is_alive()),
            'publish_rate_hz': self._publish_rate,
            'smoothing_factor': self._smoothing_factor,
            'target_speeds': list(self._target_speeds),
            'current_speeds': list(self._current_speeds),
            'last_command_time': _iso(self._last_command_time),
            'last_command_age_s': _ago(self._last_command_time),
            'last_publish_time': _iso(self._last_publish_time),
            'last_publish_age_s': _ago(self._last_publish_time),
            'last_error': self._last_error,
            'last_command_payload': list(self._last_command_payload),
            'publishers': publishers,
            'env': {
                'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
                'ROS_VERSION': os.environ.get('ROS_VERSION'),
                'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', '')[:200],
                'PYTHONPATH': os.environ.get('PYTHONPATH', '')[:200],
                'HOSTNAME': platform.node(),
            },
            'process': {
                'pid': os.getpid(),
            },
        }



