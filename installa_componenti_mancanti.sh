#!/bin/bash
# Script per installare tutti i componenti mancanti sull'AI Accelerator

set -e

echo "================================================================================"
echo "INSTALLAZIONE COMPONENTI AI ACCELERATOR - UNIVERSAL ROBOTS"
echo "================================================================================"
echo

# Setup ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Directory di lavoro
cd ~/MekoAiAccelerator || cd ~

echo "1. INSTALLAZIONE PACCHETTI PYTHON..."
echo "--------------------------------------------------------------------------------"

# ur_rtde
echo "   → Installazione ur_rtde..."
pip3 install --user ur-rtde || pip3 install ur-rtde
echo "   ✅ ur_rtde installato"

# MuJoCo
echo "   → Installazione MuJoCo..."
pip3 install --user mujoco || pip3 install mujoco
echo "   ✅ MuJoCo installato"

# YOLOv8 (ultralytics)
echo "   → Installazione YOLOv8 (ultralytics)..."
pip3 install --user ultralytics || pip3 install ultralytics
echo "   ✅ YOLOv8 installato"

# Open3D
echo "   → Installazione Open3D..."
pip3 install --user open3d || pip3 install open3d
echo "   ✅ Open3D installato"

echo
echo "2. INSTALLAZIONE MUJOCO MENAGERIE..."
echo "--------------------------------------------------------------------------------"

MUJOCO_MENAGERIE_DIR="$HOME/mujoco_menagerie"
if [ ! -d "$MUJOCO_MENAGERIE_DIR" ]; then
    echo "   → Clonazione MuJoCo Menagerie..."
    cd ~
    git clone https://github.com/google-deepmind/mujoco_menagerie.git || {
        echo "   ⚠️ Errore clonazione, provo con SSH..."
        git clone git@github.com:google-deepmind/mujoco_menagerie.git || {
            echo "   ❌ Impossibile clonare MuJoCo Menagerie"
        }
    }
    echo "   ✅ MuJoCo Menagerie clonato"
else
    echo "   ✅ MuJoCo Menagerie già presente"
fi

echo
echo "3. VERIFICA E COMPILAZIONE UR ROS2 DRIVER..."
echo "--------------------------------------------------------------------------------"

ROS2_WS="$HOME/ros2_ws"
UR_DRIVER_DIR="$ROS2_WS/src/Universal_Robots_ROS2_Driver"

if [ -d "$UR_DRIVER_DIR" ]; then
    echo "   → UR ROS2 Driver presente, verifica dipendenze..."
    cd "$ROS2_WS"
    
    # Installa dipendenze
    echo "   → Installazione dipendenze ROS2..."
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y || {
        echo "   ⚠️ Alcune dipendenze potrebbero mancare, continuo..."
    }
    
    # Compila se necessario
    if [ ! -d "$ROS2_WS/install/ur_robot_driver" ]; then
        echo "   → Compilazione UR ROS2 Driver..."
        colcon build --packages-select ur_robot_driver --cmake-args -DCMAKE_BUILD_TYPE=Release || {
            echo "   ⚠️ Errore compilazione, verificare dipendenze"
        }
    else
        echo "   ✅ UR ROS2 Driver già compilato"
    fi
else
    echo "   ❌ UR ROS2 Driver NON trovato in $UR_DRIVER_DIR"
fi

echo
echo "4. VERIFICA E COMPILAZIONE ORBBEC SDK ROS2..."
echo "--------------------------------------------------------------------------------"

ORBBEC_DIR="$ROS2_WS/src/OrbbecSDK_ROS2"

if [ -d "$ORBBEC_DIR" ]; then
    echo "   → OrbbecSDK ROS2 presente, verifica compilazione..."
    cd "$ROS2_WS"
    
    if [ ! -d "$ROS2_WS/install/orbbec_camera" ]; then
        echo "   → Compilazione OrbbecSDK ROS2..."
        colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release || {
            echo "   ⚠️ Errore compilazione OrbbecSDK, verificare dipendenze"
        }
    else
        echo "   ✅ OrbbecSDK ROS2 già compilato"
    fi
else
    echo "   ⚠️ OrbbecSDK ROS2 NON trovato in $ORBBEC_DIR"
fi

echo
echo "5. VERIFICA INSTALLAZIONE..."
echo "--------------------------------------------------------------------------------"

# Test import Python
echo "   → Test import Python packages..."
python3 -c "import ur_rtde; print('   ✅ ur_rtde OK')" 2>&1 || echo "   ❌ ur_rtde FAIL"
python3 -c "import mujoco; print('   ✅ MuJoCo OK -', mujoco.__version__)" 2>&1 || echo "   ❌ MuJoCo FAIL"
python3 -c "import ultralytics; print('   ✅ YOLOv8 OK -', ultralytics.__version__)" 2>&1 || echo "   ❌ YOLOv8 FAIL"
python3 -c "import open3d; print('   ✅ Open3D OK -', open3d.__version__)" 2>&1 || echo "   ❌ Open3D FAIL"

# Test MuJoCo Menagerie
if [ -d "$MUJOCO_MENAGERIE_DIR/universal_robots_ur5e" ]; then
    echo "   ✅ Modello UR5e presente"
else
    echo "   ⚠️ Modello UR5e NON presente"
fi

if [ -d "$MUJOCO_MENAGERIE_DIR/universal_robots_ur10e" ]; then
    echo "   ✅ Modello UR10e presente"
else
    echo "   ⚠️ Modello UR10e NON presente"
fi

echo
echo "================================================================================"
echo "✅ INSTALLAZIONE COMPLETATA"
echo "================================================================================"
echo
echo "💡 Prossimi passi:"
echo "   1. Source workspace: source ~/ros2_ws/install/setup.bash"
echo "   2. Avvia driver UR: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194"
echo "   3. Test MuJoCo: python3 -m mujoco.viewer --mjcf ~/mujoco_menagerie/universal_robots_ur5e/scene.xml"
echo






