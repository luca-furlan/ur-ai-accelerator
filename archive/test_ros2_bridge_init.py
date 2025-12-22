#!/usr/bin/env python3
"""Test ROS2 bridge initialization"""
import sys
import os
sys.path.insert(0, os.path.expanduser('~/MekoAiAccelerator'))

# Source ROS2
os.environ['ROS_DISTRO'] = 'humble'
os.environ['LD_LIBRARY_PATH'] = '/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu'

from ros2_bridge_fixed import ROS2Bridge
import time

print("Creating ROS2Bridge...")
bridge = ROS2Bridge()

print("Waiting for initialization...")
time.sleep(3)

status = bridge.get_status()
print("\n=== STATUS ===")
print(f"ROS initialized: {status['ros_initialized']}")
print(f"rclpy ok: {status['rclpy_ok']}")
print(f"Publish loop running: {status['publish_loop_running']}")
print(f"Last error: {status.get('last_error')}")
print(f"Publishers: {status['publishers']}")

if not status['ros_initialized']:
    print("\n❌ ROS2 non inizializzato!")
    print("Possibili cause:")
    print("1. rclpy non disponibile")
    print("2. ROS2 non configurato correttamente")
    print("3. Errore durante inizializzazione")
else:
    print("\n✅ ROS2 inizializzato correttamente!")











