#!/bin/bash
# Script per verificare accesso e configurazione AI Accelerator

echo "================================================================================"
echo "VERIFICA ACCESSO AI ACCELERATOR"
echo "================================================================================"
echo

# Informazioni sistema
echo "1. INFORMAZIONI SISTEMA:"
echo "--------------------------------------------------------------------------------"
echo "   Hostname: $(hostname)"
echo "   IP: $(hostname -I | awk '{print $1}')"
echo "   OS: $(lsb_release -d | cut -f2)"
echo "   Kernel: $(uname -r)"
echo "   Arch: $(uname -m)"
echo

# Verifica VNC
echo "2. STATO VNC/DESKTOP REMOTO:"
echo "--------------------------------------------------------------------------------"
if systemctl is-active --quiet vncserver@* 2>/dev/null; then
    echo "   ✅ VNC Server attivo"
    systemctl status vncserver@* --no-pager | head -5
elif pgrep -x vncserver > /dev/null; then
    echo "   ✅ VNC Server in esecuzione (processo)"
    ps aux | grep vncserver | grep -v grep | head -2
else
    echo "   ⚠️ VNC Server NON attivo"
    echo "   💡 Per attivare: vncserver :1 -geometry 1920x1080"
fi
echo

# Verifica SSH
echo "3. STATO SSH:"
echo "--------------------------------------------------------------------------------"
if systemctl is-active --quiet ssh; then
    echo "   ✅ SSH Server attivo"
    echo "   Porta: $(ss -tlnp | grep ssh | awk '{print $4}' | cut -d: -f2)"
else
    echo "   ❌ SSH Server NON attivo"
fi
echo

# Verifica ROS2
echo "4. STATO ROS2:"
echo "--------------------------------------------------------------------------------"
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "   ✅ ROS2 Humble installato"
    source /opt/ros/humble/setup.bash 2>/dev/null
    echo "   ROS_DISTRO: $ROS_DISTRO"
    echo "   ROS_VERSION: $ROS_VERSION"
else
    echo "   ❌ ROS2 NON trovato"
fi
echo

# Verifica workspace
echo "5. WORKSPACE ROS2:"
echo "--------------------------------------------------------------------------------"
if [ -d ~/ros2_ws ]; then
    echo "   ✅ Workspace trovato: ~/ros2_ws"
    if [ -d ~/ros2_ws/install ]; then
        echo "   ✅ Install directory presente"
        PKG_COUNT=$(source /opt/ros/humble/setup.bash 2>/dev/null && source ~/ros2_ws/install/setup.bash 2>/dev/null && ros2 pkg list 2>/dev/null | wc -l)
        echo "   Pacchetti installati: $PKG_COUNT"
    else
        echo "   ⚠️ Install directory NON presente (compilare workspace)"
    fi
else
    echo "   ❌ Workspace NON trovato"
fi
echo

# Verifica componenti Python
echo "6. COMPONENTI PYTHON:"
echo "--------------------------------------------------------------------------------"
python3 -c "import mujoco; print('   ✅ MuJoCo -', mujoco.__version__)" 2>&1 || echo "   ❌ MuJoCo"
python3 -c "import ultralytics; print('   ✅ YOLOv8 -', ultralytics.__version__)" 2>&1 || echo "   ❌ YOLOv8"
python3 -c "import open3d; print('   ✅ Open3D -', open3d.__version__)" 2>&1 || echo "   ❌ Open3D"
python3 -c "from ur_rtde import rtde_control; print('   ✅ ur_rtde')" 2>&1 || echo "   ⚠️ ur_rtde (verificare import)"
echo

# Verifica MuJoCo Menagerie
echo "7. MUJOCO MENAGERIE:"
echo "--------------------------------------------------------------------------------"
if [ -d ~/mujoco_menagerie ]; then
    echo "   ✅ MuJoCo Menagerie presente"
    if [ -d ~/mujoco_menagerie/universal_robots_ur5e ]; then
        echo "   ✅ Modello UR5e presente"
    fi
    if [ -d ~/mujoco_menagerie/universal_robots_ur10e ]; then
        echo "   ✅ Modello UR10e presente"
    fi
else
    echo "   ❌ MuJoCo Menagerie NON trovato"
fi
echo

# Verifica web interface
echo "8. WEB INTERFACE:"
echo "--------------------------------------------------------------------------------"
if [ -f ~/MekoAiAccelerator/remote_ur_control/web_interface.py ]; then
    echo "   ✅ Web Interface presente"
    if pgrep -f "web_interface" > /dev/null; then
        PORT=$(ss -tlnp | grep python | grep -oE ':[0-9]+' | head -1 | cut -d: -f2)
        echo "   ✅ Web Interface in esecuzione su porta: $PORT"
        echo "   URL: http://$(hostname -I | awk '{print $1}'):${PORT:-8081}"
    else
        echo "   ⚠️ Web Interface NON in esecuzione"
    fi
else
    echo "   ❌ Web Interface NON trovata"
fi
echo

# Verifica connessione robot
echo "9. CONNESSIONE ROBOT:"
echo "--------------------------------------------------------------------------------"
if ping -c 1 192.168.10.194 > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile (192.168.10.194)"
else
    echo "   ❌ Robot NON raggiungibile"
fi
echo

# Porte aperte
echo "10. PORTE APERTE:"
echo "--------------------------------------------------------------------------------"
echo "   Porte in ascolto:"
ss -tlnp | grep LISTEN | awk '{print "   - " $4}' | head -10
echo

echo "================================================================================"
echo "✅ VERIFICA COMPLETATA"
echo "================================================================================"
echo
echo "💡 Per accedere:"
echo "   SSH: ssh lab@$(hostname -I | awk '{print $1}')"
if pgrep -x vncserver > /dev/null; then
    echo "   VNC: $(hostname -I | awk '{print $1}'):5901"
fi
echo












