#!/bin/bash

# ================================================================================
# SOLUZIONE DEFINITIVA RTDE OVERFLOW
# ================================================================================
# Il problema è RTDE overflow causato da:
# 1. Web interface che usa RTDE contemporaneamente
# 2. EtherNet/IP che occupa RTDE
# 3. Altri processi RTDE attivi
# ================================================================================

set +e

ROBOT_IP="192.168.10.194"
ROBOT_TYPE="ur5e"

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
print_header "🔧 SOLUZIONE RTDE OVERFLOW"

echo "PROBLEMA:"
echo "  Pipeline producer overflowed! <RTDE Data Pipeline>"
echo "  Segmentation fault quando attiva controller"
echo ""
echo "CAUSA:"
echo "  Altri processi RTDE attivi contemporaneamente al driver ROS2"
echo ""
sleep 2

cd ~/MekoAiAccelerator/metodo_guida_pratica || exit 1

# ================================================================================
# PASSO 1: FERMA TUTTO
# ================================================================================
print_header "1. FERMA TUTTI I PROCESSI RTDE"

print_info "Fermo web interface, driver ROS2, proxy server..."
pkill -f "web_interface\|flask.*8081\|ur_robot_driver\|ur_control.launch\|external_control_proxy" 2>/dev/null
sleep 3

# Verifica processi RTDE
print_info "Verifica processi RTDE attivi..."
RTDE_PROCS=$(ps aux | grep -E "rtde|ur_rtde|30004" | grep -v grep)
if [ -n "$RTDE_PROCS" ]; then
    print_warning "Processi RTDE trovati:"
    echo "$RTDE_PROCS"
    echo ""
    print_info "Fermo processi RTDE..."
    pkill -f "rtde\|ur_rtde" 2>/dev/null
    sleep 2
else
    print_success "Nessun processo RTDE attivo"
fi

echo ""

# ================================================================================
# PASSO 2: VERIFICA PORTA 50002 LIBERA
# ================================================================================
print_header "2. VERIFICA PORTA 50002 LIBERA"

if netstat -tuln | grep -q ":50002 "; then
    print_warning "Porta 50002 ancora in uso!"
    if command -v lsof > /dev/null 2>&1; then
        echo "Processi che usano porta 50002:"
        sudo lsof -i :50002 2>/dev/null || echo "Impossibile verificare"
    fi
    print_info "Attendo 3 secondi..."
    sleep 3
    pkill -9 -f "external_control_proxy\|ur_robot_driver" 2>/dev/null
    sleep 2
fi

if netstat -tuln | grep -q ":50002 "; then
    print_error "Porta 50002 ancora occupata!"
    echo "Risolvi manualmente:"
    echo "  sudo lsof -i :50002"
    echo "  sudo kill -9 <PID>"
    exit 1
else
    print_success "Porta 50002 libera"
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
# PASSO 4: AVVIA SOLO DRIVER ROS2 (SENZA WEB INTERFACE)
# ================================================================================
print_header "4. AVVIO DRIVER ROS2 (SENZA ALTRI PROCESSI RTDE)"

print_warning "IMPORTANTE:"
echo "  - Web interface NON deve essere attiva (usa RTDE)"
echo "  - Nessun altro processo RTDE deve essere attivo"
echo "  - EtherNet/IP dovrebbe essere disabilitato sul robot"
echo ""

print_info "Avvio driver ROS2..."
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
    print_warning "Possibili cause RTDE overflow:"
    echo "  1. EtherNet/IP abilitato sul robot (occupa variabili RTDE)"
    echo "  2. Web interface attiva (usa RTDE contemporaneamente)"
    echo "  3. Altro processo RTDE attivo"
    echo ""
    print_info "Verifica log:"
    echo "  tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log | grep -E 'overflow|RTDE|Variable'"
fi

