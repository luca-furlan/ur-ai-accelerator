#!/bin/bash
# Script di test da eseguire sull'AI Accelerator

echo "=========================================="
echo "TEST SETUP AI ACCELERATOR"
echo "=========================================="

# 1. Verifica Python
echo ""
echo "1. Verifica Python..."
python3 --version
if [ $? -ne 0 ]; then
    echo "❌ Python3 non trovato!"
    exit 1
fi
echo "✅ Python OK"

# 2. Verifica Flask
echo ""
echo "2. Verifica Flask..."
python3 -c "import flask; print(f'Flask {flask.__version__}')" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️ Flask non installato - installazione..."
    python3 -m pip install --user flask
fi
echo "✅ Flask OK"

# 3. Verifica ROS2
echo ""
echo "3. Verifica ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    ros2 --version 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "✅ ROS2 Humble disponibile"
        
        # Verifica driver UR
        if [ -d ~/ros2_ws/install ]; then
            source ~/ros2_ws/install/setup.bash 2>/dev/null
            ros2 pkg list | grep ur_robot_driver > /dev/null
            if [ $? -eq 0 ]; then
                echo "✅ UR Robot Driver installato"
            else
                echo "⚠️ UR Robot Driver non trovato (opzionale)"
            fi
        else
            echo "⚠️ Workspace ROS2 non trovato (opzionale)"
        fi
    else
        echo "⚠️ ROS2 non inizializzato correttamente"
    fi
else
    echo "⚠️ ROS2 non installato (opzionale - userà fallback socket)"
fi

# 4. Verifica file
echo ""
echo "4. Verifica file..."
cd ~/MekoAiAccelerator 2>/dev/null || cd ~
if [ -f remote_ur_control/web_interface.py ]; then
    echo "✅ File web_interface.py trovato"
else
    echo "❌ File web_interface.py NON trovato!"
    echo "   Esegui il deploy prima"
    exit 1
fi

if [ -f ros2_bridge_fixed.py ]; then
    echo "✅ File ros2_bridge_fixed.py trovato"
else
    echo "⚠️ File ros2_bridge_fixed.py non trovato (opzionale)"
fi

# 5. Test import
echo ""
echo "5. Test import Python..."
python3 -c "
import sys
sys.path.insert(0, '.')
try:
    from remote_ur_control import web_interface
    print('✅ Import web_interface OK')
except Exception as e:
    print(f'❌ Errore import: {e}')
    sys.exit(1)

try:
    from ros2_bridge_fixed import ROS2Bridge
    print('✅ Import ROS2Bridge OK')
except ImportError as e:
    print(f'⚠️ ROS2Bridge non disponibile: {e}')
" 2>&1

# 6. Test connessione robot
echo ""
echo "6. Test connessione robot..."
export UR_ROBOT_IP=192.168.10.194
python3 -c "
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex(('192.168.10.194', 30002))
    sock.close()
    if result == 0:
        print('✅ Robot raggiungibile (porta 30002)')
    else:
        print('❌ Robot NON raggiungibile')
except Exception as e:
    print(f'❌ Errore connessione: {e}')
" 2>&1

echo ""
echo "=========================================="
echo "TEST COMPLETATI"
echo "=========================================="
echo ""
echo "Per avviare la web interface:"
echo "  export UR_ROBOT_IP=192.168.10.194"
echo "  export WEB_HOST=0.0.0.0"
echo "  export WEB_PORT=8080"
echo "  python3 -m remote_ur_control.web_interface"
echo ""
echo "Poi apri: http://192.168.10.191:8080"


