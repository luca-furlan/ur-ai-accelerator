from setuptools import setup

package_name = 'yolov5_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov5_detector.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/models', ['models/README.md', 'models/coco_labels.txt']),
        ('share/' + package_name + '/scripts', ['models/download_yolov5n_onnx.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luca Furlan',
    maintainer_email='lab@example.com',
    description='ROS2 node for running YOLOv5 object detection on Orbbec Gemini camera frames.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_detector_node = yolov5_detector.node:main',
        ],
    },
)
