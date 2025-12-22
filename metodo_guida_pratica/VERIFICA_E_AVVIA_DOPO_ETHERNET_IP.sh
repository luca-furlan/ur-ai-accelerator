#!/bin/bash

# Script completo: verifica stato e avvia driver dopo disabilitazione EtherNet/IP

set +e

PC_IP="192.168.10.191"
ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"
PORT="50002"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo ""
    echo "=================================================================================="
    echo "$1"
    echo "=================================================================================="
    echo ""
}

print_success() { echo -e "${GREEN}✅ $1${NC}"; }
print_error() { echo -e "${RED}❌ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠️  $1${NC}"; }
print_info() { echo -e "${BLUE}ℹ️  $1${NC}"; }

clear
print_header "🔍 VERIFICA STATO E AVVIO DRIVER ROS2"

cd ~/MekoAiAccelerator/metodo_guida_pratica || exit 1

# ================================================================================
# PASSO 1: VERIFICA CONNETTIVITÀ ROBOT
# ================================================================================
print_header "1. VERIFICA CONNETTIVITÀ ROBOT"

print_info "Test ping robot..."
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    print_success "Robot raggiungibile: $ROBOT_IP"
else
    print_error "Robot NON raggiungibile: $ROBOT_IP"
    exit 1
fi

# Verifica porte robot
print_info "Verifica porte robot..."
for PORT_ROBOT in 29999 30001 30002 30004; do
    if timeout 2 nc -zv "$ROBOT_IP" "$PORT_ROBOT" 2>&1 | grep -q "succeeded\|open"; then
        print_success "Porta $PORT_ROBOT aperta"
    else
        print_warning "Porta $PORT_ROBOT chiusa o non raggiungibile"
    fi
done

echo ""

# ================================================================================
# PASSO 2: FERMA TUTTI I PROCESSI
# ================================================================================
print_header "2. FERMA PROCESSI ESISTENTI"

print_info "Fermo web interface, driver ROS2, proxy server..."
pkill -f "web_interface\|flask.*8081\|ur_robot_driver\|ur_control.launch\|external_control_proxy" 2>/dev/null
sleep 3

# Verifica porta 50002 libera
if netstat -tuln | grep -q ":$PORT "; then
    print_warning "Porta $PORT ancora in uso!"
    if command -v lsof > /dev/null 2>&1; then
        echo "Processi che usano porta $PORT:"
        sudo lsof -i :$PORT 2>/dev/null || echo "Impossibile verificare"
    fi
    print_info "Attendo 3 secondi e riprovo..."
    sleep 3
    pkill -9 -f "external_control_proxy\|ur_robot_driver" 2>/dev/null
    sleep 2
fi

if netstat -tuln | grep -q ":$PORT "; then
    print_error "Porta $PORT ancora occupata!"
    echo "Risolvi manualmente:"
    echo "  sudo lsof -i :$PORT"
    echo "  sudo kill -9 <PID>"
    exit 1
else
    print_success "Porta $PORT libera"
fi

echo ""

# ================================================================================
# PASSO 3: CONFIGURA ROS2
# ================================================================================
print_header "3. CONFIGURAZIONE ROS2"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    print_success "ROS2 Humble configurato"
else
    print_error "ROS2 Humble non trovato!"
    exit 1
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    print_success "Workspace ROS2 configurato"
fi

echo ""

# ================================================================================
# PASSO 4: AVVIA DRIVER ROS2
# ================================================================================
print_header "4. AVVIO DRIVER ROS2"

print_warning "IMPORTANTE:"
echo "  - EtherNet/IP deve essere DISABILITATO sul robot ✅"
echo "  - Nessun altro processo RTDE deve essere attivo"
echo "  - Il driver ROS2 sarà il server sulla porta $PORT"
echo ""

print_info "Avvio driver ROS2..."
echo "Premi CTRL+C per fermare"
echo ""

# Avvia driver in foreground per vedere errori
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$ROBOT_TYPE \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false

# Se il driver si ferma
if [ $? -ne 0 ]; then
    echo ""
    print_error "Driver ROS2 terminato con errore"
    echo ""
    print_info "Verifica log:"
    echo "  tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -E 'overflow|RTDE|ERROR|FATAL'"
    echo ""
    print_warning "Se vedi ancora RTDE overflow:"
    echo "  1. Verifica EtherNet/IP è DISABILITATO sul robot"
    echo "  2. Riavvia robot dopo aver disabilitato EtherNet/IP"
    echo "  3. Verifica nessun altro processo RTDE attivo"
fi








