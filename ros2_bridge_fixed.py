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
    # CRITICAL: Set LD_LIBRARY_PATH FIRST - must include all ROS2 lib paths
    ros_lib = '/opt/ros/humble/lib'
    ros_lib_aarch64 = '/opt/ros/humble/lib/aarch64-linux-gnu'
    
    current_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_paths = []
    
    # Add aarch64 path first (required for librcl_action.so)
    if ros_lib_aarch64 not in current_ld and os.path.exists(ros_lib_aarch64):
        ld_paths.append(ros_lib_aarch64)
    
    # Add main ROS2 lib path
    if ros_lib not in current_ld:
        ld_paths.append(ros_lib)
    
    # Merge with existing paths
    if ld_paths:
        new_ld = ':'.join(ld_paths)
        if current_ld:
            os.environ['LD_LIBRARY_PATH'] = f'{new_ld}:{current_ld}'
        else:
            os.environ['LD_LIBRARY_PATH'] = new_ld
    
    if 'ROS_DISTRO' in os.environ:
        # Already sourced, just ensure paths are set
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
            # Parse environment variables - IMPORTANTE: prendi TUTTE le variabili ROS2
            for line in result.stdout.strip().split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    # Set ALL ROS2-related environment variables
                    if key.startswith('ROS_') or key in ['LD_LIBRARY_PATH', 'PYTHONPATH', 'CMAKE_PREFIX_PATH', 'PATH', 'PKG_CONFIG_PATH']:
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
                        elif key == 'PATH':
                            # Merge with existing PATH
                            current = os.environ.get('PATH', '')
                            if current:
                                os.environ[key] = f'{value}:{current}'
                            else:
                                os.environ[key] = value
                        else:
                            os.environ[key] = value
            
            # Aggiungi anche path Python ROS2 espliciti (potrebbero mancare)
            # IMPORTANTE: rclpy si trova in local/lib/python3.10/dist-packages
            ros_python_paths = [
                '/opt/ros/humble/lib/python3.10/site-packages',
                '/opt/ros/humble/local/lib/python3.10/dist-packages',  # QUI si trova rclpy!
            ]
            current_pythonpath = os.environ.get('PYTHONPATH', '')
            for ros_path in ros_python_paths:
                if os.path.exists(ros_path) and ros_path not in current_pythonpath:
                    os.environ['PYTHONPATH'] = f'{ros_path}:{current_pythonpath}' if current_pythonpath else ros_path
                    current_pythonpath = os.environ['PYTHONPATH']
            
            # Aggiungi anche a sys.path direttamente (per sicurezza)
            import sys
            for ros_path in ros_python_paths:
                if os.path.exists(ros_path) and ros_path not in sys.path:
                    sys.path.insert(0, ros_path)
            
            # Ensure ROS2 lib is in LD_LIBRARY_PATH (critical for librcl_action.so)
            # IMPORTANTE: aggiungi anche path aarch64 per librcl_action.so
            ros_lib = '/opt/ros/humble/lib'
            ros_lib_aarch64 = '/opt/ros/humble/lib/aarch64-linux-gnu'
            current_ld = os.environ.get('LD_LIBRARY_PATH', '')
            
            # Aggiungi aarch64 path se esiste e non è già presente
            if os.path.exists(ros_lib_aarch64) and ros_lib_aarch64 not in current_ld:
                os.environ['LD_LIBRARY_PATH'] = f'{ros_lib_aarch64}:{current_ld}' if current_ld else ros_lib_aarch64
                current_ld = os.environ['LD_LIBRARY_PATH']
            
            if ros_lib not in current_ld:
                os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib
        else:
            # Fallback: set common paths
            ros_lib = '/opt/ros/humble/lib'
            ros_lib_aarch64 = '/opt/ros/humble/lib/aarch64-linux-gnu'
            current_ld = os.environ.get('LD_LIBRARY_PATH', '')
            
            # Aggiungi aarch64 path se esiste
            if os.path.exists(ros_lib_aarch64) and ros_lib_aarch64 not in current_ld:
                os.environ['LD_LIBRARY_PATH'] = f'{ros_lib_aarch64}:{current_ld}' if current_ld else ros_lib_aarch64
                current_ld = os.environ['LD_LIBRARY_PATH']
            
            if ros_lib not in current_ld:
                os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{current_ld}' if current_ld else ros_lib
    except Exception as e:
        print(f'⚠️ Warning: Could not source ROS2 setup: {e}')
        # Fallback: set common paths anyway
        ros_lib = '/opt/ros/humble/lib'
        ros_lib_aarch64 = '/opt/ros/humble/lib/aarch64-linux-gnu'
        current_ld = os.environ.get('LD_LIBRARY_PATH', '')
        
        # Aggiungi aarch64 path se esiste
        if os.path.exists(ros_lib_aarch64) and ros_lib_aarch64 not in current_ld:
            os.environ['LD_LIBRARY_PATH'] = f'{ros_lib_aarch64}:{current_ld}' if current_ld else ros_lib_aarch64
            current_ld = os.environ['LD_LIBRARY_PATH']
        
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
    print(f'[OK] ROS2 available - LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "not set")[:100]}...')
except ImportError as e:
    ROS2_AVAILABLE = False
    Node = None
    # Log solo una volta all'import, non ad ogni chiamata
    if '_ros2_warn_logged' not in globals():
        globals()['_ros2_warn_logged'] = True
        print(f'[WARN] ROS2 not available: {e}')
        print(f'   LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "not set")}')
        print(f'   PYTHONPATH={os.environ.get("PYTHONPATH", "not set")[:100]}...')
        print(f'   Suggerimento: avvia web interface con: bash avvia_web_interface.sh')

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
        self._current_positions = [0.0] * 6  # Current joint positions (for trajectory control)
        self._positions_initialized = False  # Flag: posizioni lette almeno una volta?
        self._user_command_received = False  # Flag CRITICO: comando utente ricevuto almeno una volta?
        self._speed_lock = threading.Lock()
        self._publish_rate = 125.0  # Hz - Frequenza originale che funzionava (125Hz = 8ms intervals)
        self._smoothing_factor = 0.5  # Exponential smoothing: aumentato per risposta più rapida (0.5 = più reattivo, meno smooth)
        # Higher value = faster response but less smooth (0.5 = buon compromesso per controllo real-time)
        self._trajectory_duration = 0.1  # Duration of each trajectory segment (100ms per movimento fluido)
        # TEMPORANEO: usa forward_velocity_controller per evitare segfault
        # TODO: Fix segfault con scaled_joint_trajectory_controller quando attivato
        self._use_trajectory_control = False  # Usa forward_velocity_controller (più stabile)
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
            # Log solo una volta, non ad ogni chiamata
            if '_ros2_not_available_warned' not in globals():
                globals()['_ros2_not_available_warned'] = True
                print('⚠️ ROS2 not available (rclpy not found). Web interface will work but ROS2 commands will fail.')
            return
        
        try:
            # Verifica se ROS2 è già inizializzato
            # rclpy.ok() può restituire False anche se ROS2 è già inizializzato
            # Quindi proviamo sempre a inizializzare, ma gestiamo l'eccezione
            try:
                if not rclpy.ok():
                    rclpy.init()
            except RuntimeError as e:
                if 'must only be called once' in str(e) or 'already initialized' in str(e).lower():
                    # ROS2 già inizializzato - va bene, continuiamo
                    pass
                else:
                    raise
            
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
            
            # Per speedj: usa forward_velocity_controller per controllo joystick diretto
            # Questo è più adatto per controllo joystick perché:
            # - Non richiede posizioni inizializzate
            # - Pubblica direttamente velocità senza traiettorie
            # - Più reattivo per controllo real-time
            self._publishers['speedj'] = self._node.create_publisher(
                Float64MultiArray,
                '/forward_velocity_controller/commands',
                10
            )
            print('   ⚡ Using forward_velocity_controller for joystick control (direct velocity)')
            
            # Twist per controllo cartesiano - il driver UR non ha un topic twist diretto
            # Per ora usiamo solo joint control via forward_velocity_controller
            self._publishers['twist'] = None
            self._publishers['servo_twist'] = None
            
            # Stop: usa lo stesso publisher di speedj (invia velocità zero)
            self._publishers['stop'] = self._publishers['speedj']
            
            # Subscriber per leggere posizioni correnti joint (per trajectory control)
            # Crea subscriber se usiamo trajectory control OPPURE se speedj usa movej publisher
            if self._use_trajectory_control or (self._trajectory_msg_type and self._publishers.get('speedj') == self._publishers.get('movej')):
                try:
                    from sensor_msgs.msg import JointState
                    self._joint_state_sub = self._node.create_subscription(
                        JointState,
                        '/joint_states',
                        self._joint_state_callback,
                        10
                    )
                    print('   📍 Subscribed to /joint_states for current joint positions')
                    print('   ⏳ Waiting for first joint_states message...')
                except ImportError:
                    self._joint_state_sub = None
            else:
                self._joint_state_sub = None
            
            self._ros_initialized = True
            print('✅ ROS2 bridge initialized')
            if self._use_trajectory_control:
                print('   📍 Using scaled_joint_trajectory_controller for smooth, safe motion')
            else:
                print('   ⚡ Using forward_velocity_controller for direct velocity control')
            
            # Spin in background - mantiene il nodo vivo
            def spin_ros():
                try:
                    # Spin continuo per mantenere il nodo attivo
                    executor = rclpy.executors.SingleThreadedExecutor()
                    executor.add_node(self._node)
                    while rclpy.ok() and self._ros_initialized:
                        executor.spin_once(timeout_sec=0.1)
                except Exception as e:
                    print(f'⚠️ ROS2 spin thread error: {e}')
                    self._ros_initialized = False
                    self._last_error = f"ROS2 spin error: {e}"
            
            self._ros_thread = threading.Thread(target=spin_ros, daemon=True)
            self._ros_thread.start()
            
            # Start continuous publishing loop at 125Hz
            self._start_publish_loop()
            
        except Exception as e:
            print(f'⚠️ Failed to initialize ROS2: {e}')
            import traceback
            traceback.print_exc()
            self._ros_initialized = False
    
    def _joint_state_callback(self, msg):
        """Callback per aggiornare posizioni correnti joint."""
        try:
            joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            with self._speed_lock:
                positions_found = 0
                for i, name in enumerate(joint_names):
                    if name in msg.name:
                        idx = msg.name.index(name)
                        if idx < len(msg.position):
                            # Aggiorna posizione solo se è valida (non NaN, non infinito)
                            pos = msg.position[idx]
                            if pos == pos and abs(pos) < 1000:  # Controlla NaN e valori ragionevoli
                                self._current_positions[i] = pos
                                positions_found += 1
                
                if not self._positions_initialized and positions_found >= 6:
                    self._positions_initialized = True  # Posizioni inizializzate almeno una volta
                    print(f'✅ Joint positions initialized! Positions: {self._current_positions}')
        except Exception as e:
            print(f'⚠️ Error in joint_state_callback: {e}')
            import traceback
            traceback.print_exc()
    
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
            """Continuous publishing loop - frequenza dinamica."""
            publish_count = 0
            print(f'🔄 Publish loop started (target: {self._publish_rate}Hz)')
            
            while self._running and self._ros_initialized:
                # Ricalcola interval ad ogni ciclo per supportare cambio frequenza dinamico
                with self._speed_lock:
                    current_rate = self._publish_rate
                interval = max(0.001, 1.0 / current_rate)  # Minimo 1ms per evitare busy-waiting
                
                loop_start = time.time()
                try:
                    # Verifica che ROS2 sia ancora valido
                    if not rclpy.ok():
                        print('⚠️ ROS2 context invalid, stopping publish loop')
                        self._last_error = "ROS2 context invalid"
                        break
                    
                    # Verifica che il nodo sia ancora valido
                    if not self._node:
                        print('⚠️ ROS2 node destroyed, stopping publish loop')
                        self._last_error = "ROS2 node destroyed"
                        break
                    
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
                    
                    # CRITICO: NON pubblicare NULLA finché l'utente non ha dato almeno un comando esplicito
                    # Questo previene movimenti automatici all'avvio
                    if not self._user_command_received:
                        if publish_count % 100 == 0:  # Log ogni 5 secondi (20Hz * 100 = 5s)
                            print('⏳ Waiting for user command...')
                        time.sleep(interval)
                        continue
                    
                    # IMPORTANTE: NON pubblicare se tutte le velocità sono zero (o molto vicine a zero)
                    # Questo evita movimenti indesiderati quando joystick è fermo
                    max_speed = max(abs(s) for s in speeds)
                    if max_speed < 0.001:  # Se tutte velocità < 0.001 rad/s, non pubblicare
                        time.sleep(interval)
                        continue

                    # Publish current speeds - verifica validità publisher
                    publisher = self._publishers.get('speedj')
                    if publisher:
                        # Verifica che il publisher sia ancora valido
                        try:
                            # Prova a pubblicare solo se ROS2 è OK
                            if rclpy.ok() and self._node:
                                # Controlla se usa trajectory control (solo per movej, non per speedj)
                                # Per speedj usiamo sempre forward_velocity_controller (velocità diretta)
                                if self._trajectory_msg_type and publisher == self._publishers.get('movej') and publisher != self._publishers.get('speedj'):
                                    # CRITICO: NON pubblicare traiettorie finché posizioni non sono inizializzate
                                    # Questo previene segmentation fault nel controller
                                    if not self._positions_initialized:
                                        if publish_count % 50 == 0:  # Log ogni ~4 secondi (125Hz)
                                            print('⏳ Waiting for joint positions from /joint_states...')
                                        time.sleep(interval)
                                        continue
                                    
                                    # Converti velocità in traiettoria breve
                                    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
                                    from builtin_interfaces.msg import Duration
                                    import math
                                    
                                    # Verifica che le posizioni siano valide (non tutte zero se non inizializzate)
                                    with self._speed_lock:
                                        current_positions = list(self._current_positions)
                                    
                                    # Verifica che almeno una posizione sia diversa da zero (indica inizializzazione)
                                    if all(abs(p) < 0.001 for p in current_positions):
                                        if publish_count % 50 == 0:
                                            print('⏳ Positions still zero, waiting for /joint_states...')
                                        time.sleep(interval)
                                        continue
                                    
                                    # VALIDAZIONE CRITICA: Verifica che velocità siano valide (non NaN, non infiniti)
                                    if any(not math.isfinite(v) for v in speeds):
                                        if publish_count % 50 == 0:  # Log ogni ~2.5 secondi (20Hz)
                                            print(f'⚠️ Invalid speeds (NaN/Inf): {speeds}')
                                        time.sleep(interval)
                                        continue
                                    
                                    # VALIDAZIONE CRITICA: Verifica che posizioni siano valide (non NaN, non infiniti)
                                    if any(not math.isfinite(p) for p in current_positions):
                                        if publish_count % 50 == 0:  # Log ogni ~2.5 secondi (20Hz)
                                            print(f'⚠️ Invalid positions (NaN/Inf): {current_positions}')
                                        time.sleep(interval)
                                        continue
                                    
                                    # Aggiorna posizioni correnti basate su velocità SOLO se velocità non zero
                                    dt = self._trajectory_duration
                                    
                                    # VALIDAZIONE CRITICA: Verifica che durata sia valida
                                    if not math.isfinite(dt) or dt <= 0 or dt > 10.0:
                                        print(f'⚠️ Invalid trajectory duration: {dt}')
                                        dt = 0.1  # Fallback a durata sicura
                                    
                                    with self._speed_lock:
                                        for i in range(6):
                                            if abs(speeds[i]) > 0.001:  # Aggiorna solo se velocità significativa
                                                new_pos = self._current_positions[i] + speeds[i] * dt
                                                # Verifica che nuova posizione sia valida
                                                if math.isfinite(new_pos):
                                                    self._current_positions[i] = new_pos
                                                else:
                                                    print(f'⚠️ Invalid position update for joint {i}: {new_pos}')
                                    
                                    # Crea traiettoria con posizioni VALIDE
                                    traj = JointTrajectory()
                                    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                                                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                                    point = JointTrajectoryPoint()
                                    
                                    # Assicurati che posizioni siano valide prima di assegnarle
                                    final_positions = list(self._current_positions)
                                    if any(not math.isfinite(p) for p in final_positions):
                                        print(f'⚠️ Final positions invalid, using current: {final_positions}')
                                        final_positions = list(current_positions)
                                    
                                    point.positions = final_positions  # Posizioni valide dal robot
                                    point.velocities = speeds  # Velocità desiderate
                                    
                                    # Crea Duration in modo sicuro
                                    dt_sec = int(dt)
                                    dt_nsec = int((dt - dt_sec) * 1e9)
                                    # Assicurati che nanosec sia nel range valido [0, 1e9)
                                    if dt_nsec >= 1000000000:
                                        dt_sec += 1
                                        dt_nsec = 0
                                    point.time_from_start = Duration(sec=dt_sec, nanosec=dt_nsec)
                                    traj.points = [point]
                                    
                                    # Log prima pubblicazione per debug
                                    if publish_count < 5:
                                        print(f'📤 Publishing trajectory #{publish_count+1}: positions={[f"{p:.3f}" for p in point.positions[:3]]}, velocities={[f"{v:.3f}" for v in speeds[:3]]}, duration={dt_sec}.{dt_nsec:09d}s')
                                    
                                    try:
                                        publisher.publish(traj)
                                        if publish_count % 20 == 0:  # Log ogni secondo (20Hz)
                                            print(f'📤 Published {publish_count} trajectory messages (speeds: {[f"{s:.3f}" for s in speeds[:3]]})')
                                    except Exception as pub_err:
                                        print(f'⚠️ Publish error: {pub_err}')
                                        print(f'   Trajectory: positions={point.positions[:3]}, velocities={speeds[:3]}, duration={dt_sec}.{dt_nsec}')
                                        raise
                                else:
                                    # Usa forward_velocity_controller (velocità diretta)
                                    try:
                                        msg = Float64MultiArray()
                                        msg.data = speeds
                                        # Pubblica con timeout implicito (non blocca)
                                        publisher.publish(msg)
                                        # Non aspettare conferma - pubblicazione asincrona
                                    except Exception as pub_err:
                                        # Se la pubblicazione fallisce, logga ma continua
                                        if publish_count % 50 == 0:  # Log ogni ~4 secondi
                                            print(f'⚠️ Publish error (speedj): {pub_err}')
                                        # Non fermare il loop - continua a provare
                                        pass
                                
                                publish_count += 1
                                self._last_publish_time = time.time()
                                if publish_count % 20 == 0:  # Log ogni secondo (20Hz)
                                    print(f'📤 Published {publish_count} messages (current speeds: {[f"{s:.3f}" for s in speeds]})')
                            else:
                                # ROS2 non valido, ferma il loop
                                print('⚠️ ROS2 context invalid during publish')
                                self._last_error = "ROS2 context invalid during publish"
                                break
                        except Exception as pub_error:
                            # Errore durante pubblicazione (es: context invalid)
                            error_msg = str(pub_error)
                            if "context is invalid" in error_msg or "publisher's context" in error_msg:
                                print(f'⚠️ Publisher context invalid: {error_msg}')
                                self._last_error = f"Publisher context invalid: {error_msg}"
                                # Prova a reinizializzare ROS2
                                try:
                                    if rclpy.ok():
                                        self._ros_initialized = False
                                        self.ensure_ros()
                                        print('🔄 Tentativo di reinizializzazione ROS2')
                                except:
                                    pass
                                break
                            else:
                                # Altro errore, continua
                                print(f'⚠️ Publish error: {pub_error}')
                                self._last_error = str(pub_error)
                    else:
                        print('⚠️ Publisher not available')
                        break
                    
                    # Sleep for exactly 8ms (125Hz)
                    time.sleep(interval)
                except Exception as e:
                    error_msg = str(e)
                    if "context is invalid" in error_msg or "publisher's context" in error_msg:
                        print(f'❌ ROS2 context invalid in publish loop: {e}')
                        self._last_error = f"ROS2 context invalid: {error_msg}"
                        # Ferma il loop se il contesto è invalido
                        break
                    else:
                        print(f'❌ Error in publish loop: {e}')
                        self._last_error = error_msg
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
            # CRITICO: Segna che l'utente ha dato un comando esplicito
            # Questo permette al publish loop di iniziare a pubblicare
            self._user_command_received = True
            print(f'✅ User command received! Speeds: {speeds}')
            
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
            # CRITICO: Segna che l'utente ha dato un comando esplicito (stop)
            # Questo permette al publish loop di pubblicare velocità zero
            self._user_command_received = True
            
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
    
    def set_publish_rate(self, rate_hz):
        """Cambia frequenza pubblicazione dinamicamente."""
        if rate_hz < 10.0 or rate_hz > 200.0:
            raise ValueError(f"Frequenza deve essere tra 10 e 200 Hz, ricevuto: {rate_hz}")
        
        with self._speed_lock:
            old_rate = self._publish_rate
            self._publish_rate = float(rate_hz)
        
        print(f'🔄 Frequenza pubblicazione cambiata: {old_rate}Hz → {self._publish_rate}Hz')



