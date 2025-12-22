#!/bin/bash
# Avvia ROS2 Bridge e lo mantiene in esecuzione

echo "=== AVVIO ROS2 BRIDGE CONTINUO ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Kill processi esistenti
pkill -f "ros2_bridge_fixed.py" 2>/dev/null
sleep 1

# Crea script Python che mantiene il bridge attivo
cat > /tmp/run_ros2_bridge.py << 'PYEOF'
#!/usr/bin/env python3
import sys
import time
import signal
import os

# Aggiungi path
sys.path.insert(0, os.path.expanduser('~/MekoAiAccelerator'))

# Importa bridge
from ros2_bridge_fixed import ROS2Bridge

# Crea bridge
bridge = ROS2Bridge()

# Mantieni in esecuzione
def signal_handler(sig, frame):
    print('\n🛑 Fermo ROS2 Bridge...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

print("✅ ROS2 Bridge avviato e in esecuzione")
print("   Premi CTRL+C per fermare")

try:
    while True:
        time.sleep(1)
        # Verifica che sia ancora attivo
        status = bridge.get_status()
        if not status.get('ros_initialized', False):
            print("⚠️ ROS2 non inizializzato, riprovo...")
            bridge = ROS2Bridge()
except KeyboardInterrupt:
    print('\n🛑 ROS2 Bridge fermato')
except Exception as e:
    print(f'❌ Errore: {e}')
    sys.exit(1)
PYEOF

chmod +x /tmp/run_ros2_bridge.py

# Avvia in background
cd ~/MekoAiAccelerator
nohup python3 /tmp/run_ros2_bridge.py > /tmp/ros2_bridge.log 2>&1 &
BRIDGE_PID=$!

sleep 3

if ps -p $BRIDGE_PID > /dev/null; then
    echo "✅ ROS2 Bridge avviato (PID: $BRIDGE_PID)"
    echo "📋 Log: /tmp/ros2_bridge.log"
    echo ""
    echo "Verifica ultimi log:"
    tail -5 /tmp/ros2_bridge.log
    echo ""
    echo "Per fermare: kill $BRIDGE_PID"
else
    echo "❌ ROS2 Bridge non si è avviato"
    echo "📋 Controlla log:"
    cat /tmp/ros2_bridge.log
    exit 1
fi











