#!/bin/bash
# Fix problemi rimanenti installazione

set -e

echo "================================================================================"
echo "FIX INSTALLAZIONE RIMANENTE"
echo "================================================================================"
echo

source /opt/ros/humble/setup.bash 2>/dev/null || true

# 1. Fix ur_rtde - reinstall con pip3 system-wide o user
echo "1. Fix ur_rtde..."
echo "--------------------------------------------------------------------------------"
pip3 install --user --force-reinstall ur-rtde || pip3 install --force-reinstall ur-rtde
python3 -c "import ur_rtde; print('✅ ur_rtde OK')" || {
    echo "⚠️ ur_rtde ancora non funziona, verificare PATH"
    echo "   Aggiungi a ~/.bashrc: export PATH=\$HOME/.local/bin:\$PATH"
}

# 2. Compila OrbbecSDK ROS2 in ordine corretto
echo
echo "2. Compilazione OrbbecSDK ROS2..."
echo "--------------------------------------------------------------------------------"
cd ~/ros2_ws

# Prima compila orbbec_camera_msgs
if [ -d "src/OrbbecSDK_ROS2/orbbec_camera_msgs" ]; then
    echo "   → Compilazione orbbec_camera_msgs..."
    colcon build --packages-select orbbec_camera_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release || {
        echo "   ⚠️ Errore compilazione orbbec_camera_msgs"
    }
    source install/setup.bash
fi

# Poi compila orbbec_camera
if [ -d "src/OrbbecSDK_ROS2/orbbec_camera" ]; then
    echo "   → Compilazione orbbec_camera..."
    colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release || {
        echo "   ⚠️ Errore compilazione orbbec_camera"
    }
    source install/setup.bash
fi

# 3. Verifica finale
echo
echo "3. Verifica finale..."
echo "--------------------------------------------------------------------------------"
python3 -c "import ur_rtde; print('✅ ur_rtde')" 2>&1 || echo "❌ ur_rtde"
python3 -c "import mujoco; print('✅ MuJoCo -', mujoco.__version__)" 2>&1 || echo "❌ MuJoCo"
python3 -c "import ultralytics; print('✅ YOLOv8 -', ultralytics.__version__)" 2>&1 || echo "❌ YOLOv8"
python3 -c "import open3d; print('✅ Open3D -', open3d.__version__)" 2>&1 || echo "❌ Open3D"

# Verifica pacchetti ROS2
echo
echo "   → Verifica pacchetti ROS2..."
ros2 pkg list | grep -E "ur_robot_driver|orbbec" | head -5 || echo "   ⚠️ Alcuni pacchetti ROS2 non trovati"

echo
echo "================================================================================"
echo "✅ FIX COMPLETATO"
echo "================================================================================"












