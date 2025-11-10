from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('yolov5_detector')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    weights_path = LaunchConfiguration('weights_path')
    class_names_path = LaunchConfiguration('class_names_path')
    conf_threshold = LaunchConfiguration('confidence_threshold')
    iou_threshold = LaunchConfiguration('iou_threshold')
    use_cuda = LaunchConfiguration('use_cuda')
    publish_annotated = LaunchConfiguration('publish_annotated_image')
    annotated_topic = LaunchConfiguration('annotated_topic')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('weights_path', default_value=''),
        DeclareLaunchArgument('class_names_path', default_value=''),
        DeclareLaunchArgument('confidence_threshold', default_value='0.4'),
        DeclareLaunchArgument('iou_threshold', default_value='0.45'),
        DeclareLaunchArgument('use_cuda', default_value='false'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true'),
        DeclareLaunchArgument('annotated_topic', default_value='/camera/color/yolov5_annotated'),
        Node(
            package='yolov5_detector',
            executable='yolov5_detector_node',
            name='yolov5_detector',
            parameters=[
                params_file,
                {
                    'image_topic': image_topic,
                    'weights_path': weights_path,
                    'class_names_path': class_names_path,
                    'confidence_threshold': conf_threshold,
                    'iou_threshold': iou_threshold,
                    'use_cuda': use_cuda,
                    'publish_annotated_image': publish_annotated,
                    'annotated_topic': annotated_topic,
                },
            ],
        ),
    ])
