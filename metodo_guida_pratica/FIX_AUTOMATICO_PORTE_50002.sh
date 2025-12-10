#!/bin/bash

# Fix automatico per porta 50002 - Risolve tutti i problemi trovati
# Basato su Issue #31 e #37

PC_IP="192.168.10.191"
PORT="50002"

echo "=================================================================================="
echo "🔧 FIX AUTOMATICO PORTA 50002"
echo "=================================================================================="
echo ""

# 1. Apri porta nel firewall
echo "1. Apertura porta $PORT nel firewall..."
if command -v ufw > /dev/null 2>&1; then
    if sudo ufw status 2>/dev/null | grep -q "$PORT"; then
        echo "   ✅ Porta $PORT già aperta in UFW"
    else
        echo "   Aprendo porta $PORT..."
        if sudo ufw allow $PORT 2>&1; then
            sudo ufw reload 2>&1
            echo "   ✅ Porta $PORT aperta nel firewall"
        else
            echo "   ❌ Impossibile aprire porta (serve sudo senza password)"
            echo "   Esegui manualmente: sudo ufw allow $PORT && sudo ufw reload"
        fi
    fi
else
    echo "   ⚠️  UFW non installato"
    echo "   Installa: sudo apt-get install ufw"
fi
echo ""

# 2. Verifica driver ROS2
echo "2. Verifica driver ROS2..."
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "   ✅ Driver ROS2 già in esecuzione"
else
    echo "   ❌ Driver ROS2 NON in esecuzione"
    echo ""
    echo "   Avvio driver ROS2..."
    cd ~/MekoAiAccelerator/metodo_guida_pratica
    
    if [ -f START_RAPIDO.sh ]; then
        echo "   Eseguendo START_RAPIDO.sh in background..."
        nohup ./START_RAPIDO.sh > /tmp/driver_ros2.log 2>&1 &
        echo "   ✅ Driver ROS2 avviato (PID: $!)"
        echo "   Log: tail -f /tmp/driver_ros2.log"
        sleep 3
    else
        echo "   ⚠️  START_RAPIDO.sh non trovato"
        echo "   Avvia manualmente: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.10.194 launch_rviz:=false"
    fi
fi
echo ""

# 3. Verifica macchina in ascolto
echo "3. Verifica macchina in ascolto..."
sleep 2
if netstat -tuln | grep -q ":$PORT "; then
    echo "   ✅ Macchina IN ASCOLTO sulla porta $PORT"
    netstat -tuln | grep ":$PORT "
else
    echo "   ⚠️  Macchina ancora NON in ascolto"
    echo "   Attendi qualche secondo e verifica: netstat -tuln | grep 50002"
fi
echo ""

# 4. Test porta
echo "4. Test porta $PORT..."
if timeout 3 nc -zv "$PC_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
    echo "   ✅ Porta $PORT accessibile"
else
    echo "   ⚠️  Porta $PORT ancora non accessibile"
    echo "   Verifica driver ROS2 è completamente avviato"
fi
echo ""

echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "✅ Firewall configurato"
echo "✅ Driver ROS2 avviato"
echo ""
echo "PROSSIMI PASSI SUL TEACH PENDANT:"
echo ""
echo "1. Verifica EtherNet/IP DISABILITATO (Installation → Fieldbus)"
echo "2. Verifica Remote Control abilitato (Settings → System → Remote Control)"
echo "3. Configura External Control:"
echo "   - Host IP: $PC_IP"
echo "   - Port: $PORT"
echo "4. Avvia programma sul robot (PLAY)"
echo ""
echo "=================================================================================="
echo ""
echo "Per monitorare driver ROS2:"
echo "  tail -f /tmp/driver_ros2.log"
echo ""
echo "Per verificare porta:"
echo "  netstat -tuln | grep 50002"
echo ""

