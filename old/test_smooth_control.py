#!/usr/bin/env python3
"""
Test smooth control with single continuous script on robot.
"""
import socket
import time

ROBOT_IP = "192.168.10.194"
PORT = 30002

# Start continuous velocity control loop on robot
script = """
def velocity_control():
  textmsg("Starting velocity control loop")
  
  # Move forward slowly for 3 seconds
  speedl([0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 0.5, 3.0)
  
  # Stop smoothly
  stopl(1.0)
  
  textmsg("Velocity control complete")
end

velocity_control()
"""

print(f"Connecting to {ROBOT_IP}:{PORT}...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(5)
s.connect((ROBOT_IP, PORT))
print("✅ Connected!")

print("\n🚀 Sending velocity control script...")
print("   Robot will move at 50mm/s in Y direction for 3 seconds")
print("   Then stop smoothly\n")

s.sendall(script.encode('utf-8'))
print("✅ Script sent! Watch the robot...")

time.sleep(4)  # Wait for motion to complete

s.close()
print("\n✅ Done! Was it smooth?")

