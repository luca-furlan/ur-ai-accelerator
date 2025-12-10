#!/usr/bin/env python3
"""
Test ROS2 Driver UR - verifica topics e nodi
"""

import subprocess
import sys
import os
import time

def check_ros2_available() -> bool:
    """Verifica se ROS2 è disponibile"""
    try:
        result = subprocess.run(['ros2', '--version'], capture_output=True, timeout=5)
        return result.returncode == 0
    except:
        return False

def get_ros2_topics() -> list:
    """Ottiene lista topics ROS2"""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            return [t.strip() for t in result.stdout.split('\n') if t.strip()]
        return []
    except:
        return []

def get_ros2_nodes() -> list:
    """Ottiene lista nodi ROS2"""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            return [n.strip() for n in result.stdout.split('\n') if n.strip()]
        return []
    except:
        return []

def check_ur_topics(topics: list) -> list:
    """Verifica topics UR"""
    ur_keywords = ['ur', 'joint', 'servo', 'trajectory', 'controller']
    ur_topics = []
    for topic in topics:
        topic_lower = topic.lower()
        if any(keyword in topic_lower for keyword in ur_keywords):
            ur_topics.append(topic)
    return ur_topics

def check_ur_nodes(nodes: list) -> list:
    """Verifica nodi UR"""
    ur_keywords = ['ur', 'robot', 'controller']
    ur_nodes = []
    for node in nodes:
        node_lower = node.lower()
        if any(keyword in node_lower for keyword in ur_keywords):
            ur_nodes.append(node)
    return ur_nodes

def test_topic_echo(topic: str, timeout: int = 3) -> bool:
    """Test se un topic pubblica dati"""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', topic, '--once'],
            capture_output=True,
            timeout=timeout
        )
        return result.returncode == 0 and len(result.stdout) > 0
    except:
        return False

def main():
    print("=" * 60)
    print("TEST ROS2 DRIVER UR")
    print("=" * 60)
    
    # Verifica ROS2
    print("\n1. Verifica ROS2 disponibile...")
    if not check_ros2_available():
        print("❌ ROS2 NON disponibile")
        print("   Eseguire: source /opt/ros/humble/setup.bash")
        return
    print("✅ ROS2 disponibile")
    
    # Topics
    print("\n2. Verifica topics ROS2...")
    topics = get_ros2_topics()
    if topics:
        print(f"✅ Topics disponibili: {len(topics)}")
        ur_topics = check_ur_topics(topics)
        if ur_topics:
            print(f"✅ Topics UR rilevati: {len(ur_topics)}")
            print("   Topics principali:")
            for topic in ur_topics[:10]:
                print(f"     - {topic}")
        else:
            print("⚠️ Nessun topic UR rilevato (driver non avviato?)")
    else:
        print("⚠️ Nessun topic disponibile (ROS2 daemon non avviato?)")
    
    # Nodes
    print("\n3. Verifica nodi ROS2...")
    nodes = get_ros2_nodes()
    if nodes:
        print(f"✅ Nodi disponibili: {len(nodes)}")
        ur_nodes = check_ur_nodes(nodes)
        if ur_nodes:
            print(f"✅ Nodi UR rilevati: {len(ur_nodes)}")
            print("   Nodi principali:")
            for node in ur_nodes[:10]:
                print(f"     - {node}")
        else:
            print("⚠️ Nessun nodo UR rilevato (driver non avviato?)")
    else:
        print("⚠️ Nessun nodo disponibile (ROS2 daemon non avviato?)")
    
    # Test topic specifici
    print("\n4. Test topic specifici...")
    important_topics = [
        '/joint_states',
        '/servo_node/delta_twist_cmds',
        '/forward_velocity_controller/commands',
        '/scaled_joint_trajectory_controller/joint_trajectory',
    ]
    
    for topic in important_topics:
        if topic in topics:
            print(f"   Test {topic}...")
            if test_topic_echo(topic, timeout=2):
                print(f"     ✅ {topic} attivo")
            else:
                print(f"     ⚠️ {topic} presente ma non pubblica dati")
        else:
            print(f"     ⚠️ {topic} non presente")
    
    print("\n" + "=" * 60)
    print("NOTA: Per avviare il driver UR:")
    print("  ros2 launch ur_robot_driver ur_control.launch.py \\")
    print("      ur_type:=ur5e \\")
    print("      robot_ip:=192.168.10.194 \\")
    print("      launch_rviz:=false")
    print("=" * 60)

if __name__ == '__main__':
    main()





