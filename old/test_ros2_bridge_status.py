#!/usr/bin/env python3
"""Test script to verify ROS2 bridge status."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros2_bridge_fixed import ROS2Bridge

print("Creating bridge...")
bridge = ROS2Bridge()
print("Bridge created")

print("Ensuring ROS...")
result = bridge.ensure_ros()
print(f"ROS initialized: {result}")

if result:
    print(f"Running: {bridge._running}")
    print(f"ROS initialized: {bridge._ros_initialized}")
    print(f"Publish thread alive: {bridge._publish_thread.is_alive() if bridge._publish_thread else None}")
    print(f"Current speeds: {bridge._current_speeds}")
    
    # Test publishing
    print("\nTesting publish_speedj...")
    test_speeds = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    success = bridge.publish_speedj(test_speeds)
    print(f"Publish success: {success}")
    print(f"Current speeds after publish: {bridge._current_speeds}")
else:
    print("⚠️ ROS2 not initialized")


