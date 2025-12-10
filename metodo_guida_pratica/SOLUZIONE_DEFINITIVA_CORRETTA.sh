#!/bin/bash

# ================================================================================
# SOLUZIONE DEFINITIVA CORRETTA - UR5e ROS2 Driver
# ================================================================================
# Il problema NON è EtherNet/IP, ma il conflitto di porte!
# Il driver ROS2 DEVE essere il server sulla porta 50002
# Il proxy server NON serve se il driver funziona correttamente
# ================================================================================

set +e

PC_IP="192.168.10.191"
ROBOT_IP="192.168.10.194"
PORT="50002"
ROBOT_TYPE="ur5e"

# Colori
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
print_header "🚀 SOLUZIONE DEFINITIVA CORRETTA - UR5e ROS2 Driver"

echo "PROBLEMA IDENTIFICATO:"
echo "  Il driver ROS2 cerca di mettersi in ascolto sulla porta 50002"
echo "  Ma la porta è già occupata (probabilmente dal proxy server)"
echo ""
echo "SOLUZIONE:"
echo "  1. Ferma TUTTO (proxy server incluso)"
echo "  2. Assicurati che la porta 50002 sia libera"
echo "  3. Avvia SOLO il driver ROS2 (senza proxy)"
echo "  4. Il driver ROS2 sarà il server sulla porta 50002"
echo ""
sleep 3

cd ~/MekoAiAccelerator/metodo_guida_pratica || exit 1

# ================================================================================
# PASSO 1: FERMA TUTTO
# ================================================================================
print_header "1. FERMA TUTTI I PROCESSI"

print_info "Fermo proxy server, driver ROS2 e tutti i processi RTDE..."
pkill -f "external_control_proxy\|ur_robot_driver\|ur_control.launch\|ros2.*ur" 2>/dev/null
sleep 5

# Verifica porta libera
if netstat -tuln | grep -q ":$PORT "; then
    print_warning "Porta $PORT ancora in uso!"
    if command -v lsof > /dev/null 2>&1; then
        echo "Processi che usano porta $PORT:"
        sudo lsof -i :$PORT 2>/dev/null || echo "Impossibile verificare (serve sudo)"
    fi
    echo ""
    print_warning "Attendo 5 secondi e riprovo..."
    sleep 5
    pkill -9 -f "external_control_proxy\|ur_robot_driver" 2>/dev/null
    sleep 2
fi

if netstat -tuln | grep -q ":$PORT "; then
    print_error "Porta $PORT ancora occupata! Risolvi manualmente:"
    echo "  sudo lsof -i :$PORT"
    echo "  sudo kill -9 <PID>"
    exit 1
else
    print_success "Porta $PORT libera"
fi

echo ""

# ================================================================================
# PASSO 2: CONFIGURA FIREWALL
# ================================================================================
print_header "2. CONFIGURAZIONE FIREWALL"

if command -v ufw > /dev/null 2>&1; then
    if sudo ufw status 2>/dev/null | grep -q "$PORT"; then
        print_success "Porta $PORT già aperta"
    else
        print_info "Apertura porta $PORT..."
        if sudo ufw allow $PORT 2>&1; then
            sudo ufw reload 2>&1
            print_success "Porta $PORT aperta"
        else
            print_warning "Impossibile aprire porta (serve sudo senza password)"
            echo "Esegui manualmente: sudo ufw allow $PORT && sudo ufw reload"
        fi
    fi
else
    print_warning "UFW non installato"
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
# PASSO 4: AVVIA SOLO DRIVER ROS2 (SENZA PROXY)
# ================================================================================
print_header "4. AVVIO DRIVER ROS2 (SENZA PROXY)"

print_info "IMPORTANTE: Il driver ROS2 DEVE essere l'unico processo sulla porta 50002"
echo ""
print_info "Avvio driver ROS2..."
echo ""

# Avvia driver in foreground per vedere errori
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$ROBOT_TYPE \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false

# Se il driver si ferma, mostra log
if [ $? -ne 0 ]; then
    echo ""
    print_error "Driver ROS2 terminato con errore"
    echo ""
    print_info "Verifica log per errori:"
    echo "  tail -100 ~/.ros/log/latest/ur_ros2_control_node-*.log"
    echo ""
    print_warning "Possibili cause:"
    echo "  1. EtherNet/IP abilitato (occupa variabili RTDE)"
    echo "  2. Calibration mismatch"
    echo "  3. Altro processo RTDE attivo"
fi

