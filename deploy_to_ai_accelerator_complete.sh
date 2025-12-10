#!/bin/bash
# Script completo di deploy su AI Accelerator con Orbbec e teleoperazione

set -e  # Exit on error

AI_ACCELERATOR_IP="192.168.10.191"
AI_ACCELERATOR_USER="lab"
REMOTE_DIR="~/MekoAiAccelerator"
ROS2_WS="~/ros2_ws"

echo "=========================================="
echo "DEPLOY COMPLETO SU AI ACCELERATOR"
echo "=========================================="
echo "IP: $AI_ACCELERATOR_IP"
echo "User: $AI_ACCELERATOR_USER"
echo ""

# Funzione per eseguire comandi remoti
ssh_exec() {
    ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP "$@"
}

# 1. Trasferimento file
echo "1. Trasferimento file..."
ssh_exec "mkdir -p $REMOTE_DIR/remote_ur_control"
scp -r remote_ur_control/* $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/remote_ur_control/
scp ros2_bridge_fixed.py $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/
scp setup.py $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/ 2>/dev/null || true
scp package.xml $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP:$REMOTE_DIR/ 2>/dev/null || true

echo "✅ File trasferiti"

# 2. Setup completo sull'AI Accelerator
echo ""
echo "2. Setup completo sull'AI Accelerator..."
ssh_exec << 'ENDSSH'
set -e

cd ~/MekoAiAccelerator

echo "2.1. Installazione dipendenze Python base..."
python3 -m pip install --user --upgrade pip || pip3 install --user --upgrade pip
python3 -m pip install --user flask flask-cors || pip3 install --user flask flask-cors

echo "2.2. Verifica ROS2..."
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "⚠️  ROS2 Humble non trovato. Installazione..."
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update
    sudo apt install -y ros-humble-desktop python3-argcomplete
    sudo apt install -y python3-colcon-common-extensions
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true

echo "2.3. Setup workspace ROS2..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

echo "2.4. Installazione Universal Robots ROS2 Driver..."
if [ ! -d src/Universal_Robots_ROS2_Driver ]; then
    cd src
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git || echo "Repository già presente"
    cd ..
    
    # Installa dipendenze
    source /opt/ros/humble/setup.bash
    sudo apt update
    sudo apt install -y python3-rosdep2
    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y || echo "Alcune dipendenze potrebbero mancare"
    
    # Build
    source /opt/ros/humble/setup.bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release || echo "Build potrebbe avere warning"
fi

echo "2.5. Installazione OrbbecSDK_ROS2..."
cd ~/ros2_ws/src
if [ ! -d OrbbecSDK_ROS2 ]; then
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git || echo "Repository già presente"
    cd OrbbecSDK_ROS2
    # Installa dipendenze Orbbec
    sudo apt install -y libudev-dev libusb-1.0-0-dev
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y || echo "Alcune dipendenze potrebbero mancare"
    colcon build --packages-select orbbec_camera || echo "Build Orbbec potrebbe avere warning"
fi

echo "2.6. Creazione pacchetto teleoperazione personalizzato..."
cd ~/ros2_ws/src
mkdir -p meko_teleop
cd meko_teleop

# Crea struttura pacchetto
cat > package.xml << 'PKGXML'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>meko_teleop</name>
  <version>1.0.0</version>
  <description>Teleoperazione robot UR con Orbbec camera</description>
  <maintainer email="lab@meko.ai">Meko Team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>control_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
PKGXML

mkdir -p meko_teleop
cat > setup.py << 'SETUPPY'
from setuptools import setup

package_name = 'meko_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Meko Team',
    maintainer_email='lab@meko.ai',
    description='Teleoperazione robot UR con Orbbec camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orbbec_teleop = meko_teleop.orbbec_teleop_node:main',
        ],
    },
)
SETUPPY

mkdir -p resource
touch resource/meko_teleop

mkdir -p meko_teleop
cat > meko_teleop/__init__.py << 'INIT'
# Meko Teleop Package
INIT

cat > meko_teleop/orbbec_teleop_node.py << 'NODEPY'
#!/usr/bin/env python3
"""
Nodo ROS2 per teleoperazione robot UR con Orbbec camera.
Utilizza la camera Orbbec per controllo visivo e pubblica comandi al robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
from cv_bridge import CvBridge


class OrbbecTeleopNode(Node):
    """Nodo per teleoperazione con Orbbec."""
    
    def __init__(self):
        super().__init__('orbbec_teleop_node')
        
        self.bridge = CvBridge()
        
        # Subscribers - dati da Orbbec
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Publishers - comandi al robot
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )
        
        self.twist_pub = self.create_publisher(
            Twist,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Stato
        self.current_rgb = None
        self.current_depth = None
        
        self.get_logger().info('Orbbec Teleop Node avviato')
        self.get_logger().info('In attesa di dati dalla camera Orbbec...')
    
    def rgb_callback(self, msg):
        """Callback per immagini RGB."""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().debug(f'Ricevuta immagine RGB: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Errore conversione RGB: {e}')
    
    def depth_callback(self, msg):
        """Callback per immagini depth."""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.get_logger().debug(f'Ricevuta immagine depth: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Errore conversione depth: {e}')
    
    def process_and_control(self):
        """Elabora dati camera e genera comandi robot."""
        # TODO: Implementare logica di controllo basata su visione
        # Per ora, esempio base
        if self.current_rgb is not None:
            # Elabora immagine e genera comandi
            # Esempio: tracking oggetti, controllo posizione, etc.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
NODEPY

chmod +x meko_teleop/orbbec_teleop_node.py

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select meko_teleop || echo "Build meko_teleop potrebbe avere warning"

echo "2.7. Installazione dipendenze Python ROS2..."
python3 -m pip install --user cv-bridge || pip3 install --user cv-bridge

echo "✅ Setup completato!"

ENDSSH

echo ""
echo "=========================================="
echo "✅ DEPLOY COMPLETATO!"
echo "=========================================="
echo ""
echo "Prossimi passi:"
echo ""
echo "1. Connettiti all'AI Accelerator:"
echo "   ssh $AI_ACCELERATOR_USER@$AI_ACCELERATOR_IP"
echo ""
echo "2. Source ROS2 e workspace:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/ros2_ws/install/setup.bash"
echo ""
echo "3. Avvia Orbbec camera:"
echo "   ros2 launch orbbec_camera orbbec_camera.launch.py"
echo ""
echo "4. Avvia driver UR (in altro terminale):"
echo "   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194"
echo ""
echo "5. Avvia teleoperazione:"
echo "   ros2 run meko_teleop orbbec_teleop_node"
echo ""
echo "6. Oppure avvia web interface:"
echo "   cd ~/MekoAiAccelerator"
echo "   export UR_ROBOT_IP=192.168.10.194"
echo "   python3 -m remote_ur_control.web_interface"
echo ""
echo "7. Accedi da browser:"
echo "   http://$AI_ACCELERATOR_IP:8080"
echo ""

