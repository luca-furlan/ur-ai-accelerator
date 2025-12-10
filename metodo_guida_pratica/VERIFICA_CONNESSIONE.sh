#!/bin/bash

echo "=================================================================================="
echo "🔍 VERIFICA CONNESSIONE ROBOT UR5e"
echo "=================================================================================="
echo ""

# Colori
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ROBOT_IP="192.168.10.194"
ROBOT_PORT="50002"

echo "📡 Verifica connessione di rete al robot..."
if ping -c 2 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo -e "${GREEN}✅ Robot raggiungibile su $ROBOT_IP${NC}"
else
    echo -e "${RED}❌ Robot NON raggiungibile su $ROBOT_IP${NC}"
    echo "   Verifica:"
    echo "   - Robot acceso e connesso alla rete"
    echo "   - IP robot corretto: $ROBOT_IP"
    exit 1
fi

echo ""
echo "🔌 Verifica porta 50002 (External Control)..."
if timeout 3 bash -c "</dev/tcp/$ROBOT_IP/$ROBOT_PORT" 2>/dev/null; then
    echo -e "${GREEN}✅ Porta $ROBOT_PORT APERTA e in ascolto!${NC}"
    echo "   Il robot è pronto per ricevere connessioni"
else
    echo -e "${RED}❌ Porta $ROBOT_PORT NON raggiungibile${NC}"
    echo "   Verifica sul Teach Pendant:"
    echo "   - Programma con External Control è in PLAYING?"
    echo "   - IP Host configurato: 192.168.10.191"
    echo "   - Porta configurata: 50002"
    exit 1
fi

echo ""
echo "🤖 Verifica processi ROS2..."
if pgrep -f "ur_robot_driver" > /dev/null; then
    echo -e "${GREEN}✅ Driver UR Robot attivo${NC}"
    ps aux | grep "ur_robot_driver" | grep -v grep
else
    echo -e "${YELLOW}⚠️  Driver UR Robot NON attivo${NC}"
    echo "   Esegui: ./START_RAPIDO.sh o ./SETUP_COMPLETO.sh"
fi

echo ""
echo "📊 Verifica topic ROS2..."
if command -v ros2 > /dev/null 2>&1; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    TOPICS=$(timeout 2 ros2 topic list 2>/dev/null)
    if [ $? -eq 0 ] && [ ! -z "$TOPICS" ]; then
        echo -e "${GREEN}✅ Topic ROS2 disponibili:${NC}"
        echo "$TOPICS" | head -10
    else
        echo -e "${YELLOW}⚠️  Nessun topic ROS2 disponibile${NC}"
        echo "   Il driver potrebbe non essere avviato"
    fi
else
    echo -e "${YELLOW}⚠️  ROS2 non installato o non nel PATH${NC}"
fi

echo ""
echo "=================================================================================="
echo "📋 RIEPILOGO STATO"
echo "=================================================================================="
echo ""
echo "Robot IP: $ROBOT_IP"
echo "Porta: $ROBOT_PORT"
echo ""

if ping -c 1 -W 1 $ROBOT_IP > /dev/null 2>&1 && timeout 2 bash -c "</dev/tcp/$ROBOT_IP/$ROBOT_PORT" 2>/dev/null; then
    echo -e "${GREEN}✅ CONNESSIONE OK - Robot pronto!${NC}"
    echo ""
    echo "Prossimi passi:"
    echo "1. Se driver non attivo: ./START_RAPIDO.sh"
    echo "2. Test movimento: ./TEST_MOVIMENTO.sh"
    echo "3. Verifica topic: ros2 topic list"
else
    echo -e "${RED}❌ CONNESSIONE NON OK${NC}"
    echo ""
    echo "Verifica sul Teach Pendant:"
    echo "- Programma in PLAYING?"
    echo "- External Control configurato correttamente?"
fi

echo ""

