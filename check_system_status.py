#!/usr/bin/env python3
"""Check complete system status."""
import sys
import os

print("=" * 60)
print("SYSTEM STATUS CHECK")
print("=" * 60)

# Check ROS2 bridge import
print("\n1. Checking ROS2 bridge import...")
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from ros2_bridge_fixed import ROS2Bridge, ROS2_AVAILABLE
    print(f"   ✅ ROS2 bridge imported successfully")
    print(f"   ✅ ROS2_AVAILABLE = {ROS2_AVAILABLE}")
except ImportError as e:
    print(f"   ❌ Failed to import ROS2 bridge: {e}")
    sys.exit(1)

# Check ROS2 bridge initialization
print("\n2. Checking ROS2 bridge initialization...")
try:
    bridge = ROS2Bridge()
    print(f"   ✅ Bridge created")
    
    if bridge.ensure_ros():
        print(f"   ✅ ROS2 initialized")
        print(f"   ✅ Running: {bridge._running}")
        print(f"   ✅ Publish thread alive: {bridge._publish_thread.is_alive() if bridge._publish_thread else False}")
    else:
        print(f"   ❌ ROS2 not initialized")
except Exception as e:
    print(f"   ❌ Error: {e}")
    import traceback
    traceback.print_exc()

# Test publishing
print("\n3. Testing publish_speedj...")
try:
    test_speeds = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    success = bridge.publish_speedj(test_speeds)
    print(f"   ✅ Publish success: {success}")
    print(f"   ✅ Current speeds: {bridge._current_speeds}")
except Exception as e:
    print(f"   ❌ Error: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
print("STATUS CHECK COMPLETE")
print("=" * 60)


