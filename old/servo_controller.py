#!/usr/bin/env python3
"""
Servo controller using continuous loop on robot side.
This approach sends ONE script that runs in loop reading from registers.
Much smoother than sending individual commands.
"""
import socket
import time
from dataclasses import dataclass

ROBOT_IP = "192.168.10.194"
URSCRIPT_PORT = 30002


@dataclass
class ServoController:
    """Controller that uses servoj with register-based velocity control."""
    
    robot_ip: str = ROBOT_IP
    port: int = URSCRIPT_PORT
    
    def __post_init__(self):
        self.sock = None
    
    def connect(self):
        """Connect to robot."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        self.sock.connect((self.robot_ip, self.port))
        print(f"✅ Connected to {self.robot_ip}")
    
    def start_servo_mode(self):
        """Start continuous servo loop on robot that reads velocities from registers."""
        script = """
def servo_loop():
  # Initialize
  global servo_running = True
  t = 0.008  # 8ms = 125Hz
  lookahead = 0.1
  gain = 300
  
  while servo_running:
    # Read velocities from input registers (0-5)
    vx = read_input_float_register(0)
    vy = read_input_float_register(1) 
    vz = read_input_float_register(2)
    wrx = read_input_float_register(3)
    wry = read_input_float_register(4)
    wrz = read_input_float_register(5)
    
    # Get current pose
    current_pose = get_actual_tcp_pose()
    
    # Calculate target pose (small increment based on velocities)
    target_pose = pose_add(current_pose, p[vx*t, vy*t, vz*t, wrx*t, wry*t, wrz*t])
    
    # Convert to joint space
    target_q = get_inverse_kin(target_pose)
    
    # Servo to target
    servoj(target_q, t=t, lookahead_time=lookahead, gain=gain)
  end
end

servo_loop()
"""
        if not self.sock:
            self.connect()
        
        print("🚀 Starting servo mode on robot...")
        self.sock.sendall(script.encode('utf-8'))
        print("✅ Servo mode active! Robot will now read from registers.")
        time.sleep(0.5)
    
    def set_cartesian_velocity(self, vx=0, vy=0, vz=0, wrx=0, wry=0, wrz=0):
        """Set cartesian velocities by writing to input registers."""
        # Send register writes via secondary interface or RTDE
        # For now, this is a placeholder - needs RTDE or Dashboard
        print(f"Set velocity: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
    
    def stop_servo_mode(self):
        """Stop servo loop."""
        script = "global servo_running = False\n"
        if self.sock:
            self.sock.sendall(script.encode('utf-8'))
            print("⏹️  Servo mode stopped")
    
    def close(self):
        """Close connection."""
        if self.sock:
            self.sock.close()
            self.sock = None


if __name__ == "__main__":
    controller = ServoController()
    
    try:
        controller.connect()
        controller.start_servo_mode()
        
        print("\n⚠️  Servo mode running on robot!")
        print("⚠️  To control, you need to write to input_float_register(0-5)")
        print("⚠️  This requires RTDE or Dashboard client")
        print("\nPress Ctrl+C to stop...")
        
        # Keep alive
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        controller.stop_servo_mode()
        controller.close()
        print("✅ Done")

