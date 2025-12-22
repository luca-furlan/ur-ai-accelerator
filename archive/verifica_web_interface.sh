#!/bin/bash
# Script per verificare che la web interface funzioni correttamente

echo "=================================================================================="
echo "🔍 VERIFICA WEB INTERFACE"
echo "=================================================================================="
echo ""

# Verifica processo
echo "1. Verifica processo web interface..."
if pgrep -f "web_interface|flask.*8080" > /dev/null; then
    echo "   ✅ Web interface in esecuzione"
    ps aux | grep -E "web_interface|flask.*8080" | grep -v grep | head -1
else
    echo "   ❌ Web interface NON in esecuzione"
fi
echo ""

# Verifica porta
echo "2. Verifica porta 8080..."
if netstat -tuln 2>/dev/null | grep -q ":8080 "; then
    echo "   ✅ Porta 8080 in ascolto"
    netstat -tuln | grep ":8080 "
else
    echo "   ❌ Porta 8080 NON in ascolto"
fi
echo ""

# Verifica ROS2 bridge
echo "3. Verifica ROS2 bridge..."
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

if python3 -c "import rclpy; print('OK')" 2>/dev/null; then
    echo "   ✅ ROS2 disponibile"
    
    # Verifica topic
    if timeout 2 ros2 topic list 2>/dev/null | grep -q "forward_velocity_controller/commands"; then
        echo "   ✅ Topic /forward_velocity_controller/commands esiste"
        
        # Verifica publisher/subscriber
        INFO=$(timeout 2 ros2 topic info /forward_velocity_controller/commands 2>/dev/null)
        if echo "$INFO" | grep -q "Publisher count: 1"; then
            echo "   ✅ Publisher attivo (web interface)"
        fi
        if echo "$INFO" | grep -q "Subscription count: 1"; then
            echo "   ✅ Subscriber attivo (controller)"
        fi
    else
        echo "   ⚠️  Topic non trovato (driver ROS2 potrebbe non essere attivo)"
    fi
else
    echo "   ⚠️  ROS2 non disponibile"
fi
echo ""

# Test connessione HTTP
echo "4. Test connessione HTTP..."
if curl -s -o /dev/null -w "%{http_code}" http://localhost:8080/api/status 2>/dev/null | grep -q "200"; then
    echo "   ✅ Web interface risponde"
else
    echo "   ❌ Web interface NON risponde"
fi
echo ""

echo "=================================================================================="
echo "📋 PROSSIMI PASSI"
echo "=================================================================================="
echo ""
echo "1. Apri browser: http://192.168.10.191:8080"
echo "2. Muovi il joystick"
echo "3. Verifica messaggi ROS2:"
echo "   ros2 topic echo /forward_velocity_controller/commands"
echo "4. Se il robot non si muove, esegui:"
echo "   ./test_controller_diretto.sh"
echo ""







