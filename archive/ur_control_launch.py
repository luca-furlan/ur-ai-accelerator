#!/usr/bin/env python3
"""
Launch file for UR robot control with ROS2 and web interface.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get robot IP from environment or use default
    robot_ip = os.environ.get('UR_ROBOT_IP', '192.168.10.194')
    
    return LaunchDescription([
        # UR Controller Node
        Node(
            package='remote_ur_control_ros2',
            executable='ur_controller_node',
            name='ur_controller_node',
            parameters=[{
                'robot_ip': robot_ip,
                'frequency': 125.0,
                'velocity': 0.2,
                'acceleration': 0.7,
                'publish_rate': 50.0,
            }],
            output='screen',
        ),
        
        # Web Interface (Flask)
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'remote_ur_control.web_interface_ros2'
            ],
            cwd=os.path.expanduser('~/projects/ur-ai-accelerator-remote/remote_ur_control'),
            env={
                'UR_ROBOT_IP': robot_ip,
                'WEB_HOST': '0.0.0.0',
                'WEB_PORT': '8080',
                'WEB_DEBUG': '0',
            },
            output='screen',
        ),
    ])






















