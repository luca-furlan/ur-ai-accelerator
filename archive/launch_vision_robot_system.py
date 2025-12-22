#!/usr/bin/env python3
"""
Launch file completo per sistema Vision + Robot integrato

Avvia:
1. Driver camera Orbecc
2. Vision YOLO Detector
3. MoveIt Vision Controller
4. Vision Robot Coordinator
5. (Opzionale) Web Interface

Uso:
    ros2 launch launch_vision_robot_system.py
    
Oppure con parametri:
    ros2 launch launch_vision_robot_system.py robot_ip:=192.168.10.194 auto_mode:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    """Genera launch description completa"""
    
    # Argomenti launch
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.10.194',
        description='IP del robot UR'
    )
    
    auto_mode_arg = DeclareLaunchArgument(
        'auto_mode',
        default_value='false',
        description='Abilita movimento automatico verso oggetti rilevati'
    )
    
    enable_robot_control_arg = DeclareLaunchArgument(
        'enable_robot_control',
        default_value='true',
        description='Abilita controllo robot (false per solo detection)'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='Modello YOLO da usare (n=nano, s=small, m=medium, l=large)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Soglia confidence per detections YOLO'
    )
    
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Avvia driver camera Orbecc'
    )
    
    # Configurazioni
    robot_ip = LaunchConfiguration('robot_ip')
    auto_mode = LaunchConfiguration('auto_mode')
    enable_robot_control = LaunchConfiguration('enable_robot_control')
    yolo_model = LaunchConfiguration('yolo_model')
    confidence = LaunchConfiguration('confidence')
    launch_camera = LaunchConfiguration('launch_camera')
    
    # ========================================
    # 1. CAMERA ORBECC DRIVER
    # ========================================
    # Nota: assumiamo che orbecc_camera sia installato
    # ros2 launch orbecc_camera gemini_330_series.launch.py
    
    camera_node = ExecuteProcess(
        condition=IfCondition(launch_camera),
        cmd=[
            'ros2', 'launch',
            'orbecc_camera', 'gemini_330_series.launch.py'
        ],
        output='screen',
        name='orbecc_camera'
    )
    
    # ========================================
    # 2. VISION YOLO DETECTOR
    # ========================================
    vision_detector_node = Node(
        package='remote_ur_control_ros2',  # o il nome del tuo package
        executable='vision_yolo_detector.py',
        name='vision_yolo_detector',
        output='screen',
        parameters=[{
            'camera_rgb_topic': '/camera/color/image_raw',
            'camera_depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'yolo_model': yolo_model,
            'confidence_threshold': confidence,
            'publish_rate': 10.0,
            'enable_visualization': True
        }],
        emulate_tty=True
    )
    
    # ========================================
    # 3. MOVEIT VISION CONTROLLER
    # ========================================
    moveit_controller_node = Node(
        package='remote_ur_control_ros2',
        executable='moveit_vision_controller.py',
        name='moveit_vision_controller',
        output='screen',
        parameters=[{
            'planning_group': 'ur_manipulator',
            'end_effector_link': 'tool0',
            'reference_frame': 'base_link',
            'target_classes': ['bottle', 'cup', 'cell phone', 'person', 'book'],
            'approach_offset': 0.15,
            'auto_mode': auto_mode,
            'safety_distance': 0.1
        }],
        emulate_tty=True
    )
    
    # ========================================
    # 4. VISION ROBOT COORDINATOR
    # ========================================
    coordinator_node = Node(
        package='remote_ur_control_ros2',
        executable='vision_robot_coordinator.py',
        name='vision_robot_coordinator',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
            'enable_robot_control': enable_robot_control,
            'safe_mode': True,
            'auto_pick': auto_mode
        }],
        emulate_tty=True
    )
    
    # ========================================
    # 5. RVIZ per visualizzazione (opzionale)
    # ========================================
    # TODO: aggiungere config RViz con camera view + robot model
    
    # Ritorna launch description
    return LaunchDescription([
        # Argomenti
        robot_ip_arg,
        auto_mode_arg,
        enable_robot_control_arg,
        yolo_model_arg,
        confidence_arg,
        launch_camera_arg,
        
        # Nodi
        camera_node,
        vision_detector_node,
        moveit_controller_node,
        coordinator_node,
    ])


if __name__ == '__main__':
    generate_launch_description()




