from setuptools import setup

package_name = 'yolov5_xyz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov5_xyz.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luca Furlan',
    maintainer_email='lab@example.com',
    description='Compute XYZ positions from YOLO detections and depth image.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_xyz_node = yolov5_xyz.node:main',
        ],
    },
)
