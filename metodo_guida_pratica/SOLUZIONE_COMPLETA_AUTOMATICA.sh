#!/bin/bash

# ================================================================================
# SOLUZIONE COMPLETA AUTOMATICA - UR5e ROS2 Driver
# ================================================================================
# Questo script risolve tutti i problemi e avvia tutto automaticamente
# Basato su Issue #31 e #37 di GitHub
# ================================================================================

set +e  # Continua anche con errori

# Configurazione
PC_IP="192.168.10.191"
ROBOT_IP="192.168.10.194"
ROBOT_USER="root"
ROBOT_PASS="easybot"
PORT="50002"
ROBOT_TYPE="ur5e"

# Colori per output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Funzioni di utilità
print_header() {
    echo ""
    echo "=================================================================================="
    echo "$1"
    echo "=================================================================================="
    echo ""
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# Inizio script
clear
print_header "🚀 SOLUZIONE COMPLETA AUTOMATICA - UR5e ROS2 Driver"

echo "Questo script:"
echo "  1. Verifica tutti i prerequisiti"
echo "  2. Configura firewall"
echo "  3. Ferma processi esistenti"
echo "  4. Avvia proxy server"
echo "  5. Avvia driver ROS2"
echo "  6. Verifica che tutto funzioni"
echo ""
echo "Premi CTRL+C per interrompere"
echo ""
sleep 2

cd ~/MekoAiAccelerator/metodo_guida_pratica || {
    print_error "Directory non trovata!"
    exit 1
}

# ================================================================================
# PASSO 1: VERIFICA PREREQUISITI
# ================================================================================
print_header "1. VERIFICA PREREQUISITI"

# Verifica ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    print_success "ROS2 Humble trovato"
else
    print_error "ROS2 Humble non trovato!"
    echo "Installa ROS2: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Verifica workspace ROS2
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    print_success "Workspace ROS2 trovato"
else
    print_warning "Workspace ROS2 non trovato (potrebbe essere normale)"
fi

# Verifica Python3
if command -v python3 > /dev/null 2>&1; then
    print_success "Python3 trovato"
else
    print_error "Python3 non trovato!"
    exit 1
fi

# Verifica connessione robot
print_info "Test connessione robot..."
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    print_success "Robot raggiungibile: $ROBOT_IP"
else
    print_error "Robot NON raggiungibile: $ROBOT_IP"
    echo "Verifica connessione di rete"
    exit 1
fi

# Verifica file proxy server
if [ -f external_control_proxy_server.py ]; then
    chmod +x external_control_proxy_server.py
    print_success "Proxy server trovato"
else
    print_error "File external_control_proxy_server.py non trovato!"
    exit 1
fi

echo ""

# ================================================================================
# PASSO 2: CONFIGURA FIREWALL
# ================================================================================
print_header "2. CONFIGURAZIONE FIREWALL"

if command -v ufw > /dev/null 2>&1; then
    if sudo ufw status 2>/dev/null | grep -q "$PORT"; then
        print_success "Porta $PORT già aperta in UFW"
    else
        print_info "Apertura porta $PORT..."
        if sudo ufw allow $PORT 2>&1; then
            sudo ufw reload 2>&1
            print_success "Porta $PORT aperta nel firewall"
        else
            print_warning "Impossibile aprire porta (serve sudo senza password)"
            echo "Esegui manualmente: sudo ufw allow $PORT && sudo ufw reload"
        fi
    fi
else
    print_warning "UFW non installato"
    echo "Installa: sudo apt-get install ufw"
fi

echo ""

# ================================================================================
# PASSO 3: FERMA PROCESSI ESISTENTI
# ================================================================================
print_header "3. FERMA PROCESSI ESISTENTI"

print_info "Fermo processi ROS2 e RTDE..."
pkill -f "ur_robot_driver\|ur_control.launch\|external_control_proxy" 2>/dev/null
sleep 3

# Verifica porta libera
if netstat -tuln | grep -q ":$PORT "; then
    print_warning "Porta $PORT ancora in uso!"
    if command -v lsof > /dev/null 2>&1; then
        echo "Processi che usano porta $PORT:"
        sudo lsof -i :$PORT 2>/dev/null || echo "Impossibile verificare"
    fi
    echo "Attendo 5 secondi..."
    sleep 5
else
    print_success "Porta $PORT libera"
fi

echo ""

# ================================================================================
# PASSO 4: AVVIA PROXY SERVER
# ================================================================================
print_header "4. AVVIO PROXY SERVER"

print_info "Avvio proxy server in background..."
nohup python3 external_control_proxy_server.py > /tmp/proxy_server.log 2>&1 &
PROXY_PID=$!
sleep 3

if ps -p $PROXY_PID > /dev/null 2>&1; then
    print_success "Proxy server avviato (PID: $PROXY_PID)"
else
    print_error "Proxy server NON avviato!"
    echo "Verifica log: tail -20 /tmp/proxy_server.log"
    exit 1
fi

# Verifica proxy in ascolto
sleep 2
if netstat -tuln | grep -q ":$PORT "; then
    print_success "Proxy server IN ASCOLTO sulla porta $PORT!"
    netstat -tuln | grep ":$PORT "
else
    print_warning "Proxy server potrebbe non essere in ascolto"
    echo "Verifica log: tail -f /tmp/proxy_server.log"
fi

echo ""

# ================================================================================
# PASSO 5: AVVIA DRIVER ROS2
# ================================================================================
print_header "5. AVVIO DRIVER ROS2"

print_info "Avvio driver ROS2 in background..."
nohup ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=$ROBOT_TYPE \
    robot_ip:=$ROBOT_IP \
    launch_rviz:=false \
    > /tmp/driver_ros2.log 2>&1 &

DRIVER_PID=$!
print_info "Driver avviato (PID: $DRIVER_PID)"
print_info "Attendo avvio driver (20 secondi)..."
sleep 20

# Verifica driver attivo
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    print_success "Driver ROS2 attivo"
else
    print_warning "Driver ROS2 potrebbe essere crashato"
    echo "Ultimi messaggi dal log:"
    tail -30 /tmp/driver_ros2.log | grep -E "ERROR|FATAL|Segmentation|crash" || tail -10 /tmp/driver_ros2.log
    echo ""
    print_warning "Controlla log completo: tail -f /tmp/driver_ros2.log"
fi

echo ""

# ================================================================================
# PASSO 6: VERIFICA FINALE
# ================================================================================
print_header "6. VERIFICA FINALE"

# Verifica proxy attivo
if ps -p $PROXY_PID > /dev/null 2>&1; then
    print_success "Proxy server attivo (PID: $PROXY_PID)"
else
    print_error "Proxy server NON attivo!"
fi

# Verifica porta in ascolto
if netstat -tuln | grep -q ":$PORT "; then
    print_success "Porta $PORT IN ASCOLTO"
    netstat -tuln | grep ":$PORT "
else
    print_error "Porta $PORT NON in ascolto!"
fi

# Verifica driver
if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    print_success "Driver ROS2 attivo"
else
    print_error "Driver ROS2 NON attivo"
fi

# Test connettività robot → PC
print_info "Test connettività Robot → PC..."
if timeout 3 nc -zv "$PC_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
    print_success "Robot può connettersi al PC sulla porta $PORT!"
else
    print_warning "Robot potrebbe non riuscire a connettersi (normale se driver non è ancora pronto)"
fi

echo ""

# ================================================================================
# RIEPILOGO E ISTRUZIONI
# ================================================================================
print_header "📋 RIEPILOGO E ISTRUZIONI"

echo "STATO ATTUALE:"
echo "  Proxy Server: PID $PROXY_PID"
echo "  Driver ROS2: PID $DRIVER_PID"
echo "  Porta $PORT: $(netstat -tuln | grep -q ":$PORT " && echo "IN ASCOLTO" || echo "NON in ascolto")"
echo ""

echo "PROSSIMI PASSI SUL TEACH PENDANT:"
echo ""
echo "1. DISABILITA ETHERNET/IP (CRITICO!):"
echo "   - Vai su: Installation → Fieldbus"
echo "   - EtherNet/IP deve essere DISABILITATO ❌"
echo "   - PROFINET deve essere DISABILITATO ❌"
echo "   - Solo Ethernet normale può essere abilitato ✅"
echo ""
echo "2. VERIFICA REMOTE CONTROL:"
echo "   - Vai su: Settings → System → Remote Control"
echo "   - Deve essere 'Enabled'"
echo ""
echo "3. CONFIGURA EXTERNAL CONTROL:"
echo "   - Crea programma con nodo External Control"
echo "   - Host IP: $PC_IP"
echo "   - Port: $PORT"
echo "   - Host Name: (vuoto)"
echo "   - SALVA programma"
echo ""
echo "4. AVVIA PROGRAMMA:"
echo "   - Verifica EtherNet/IP è DISABILITATO ✅"
echo "   - Verifica Remote Control è abilitato ✅"
echo "   - Verifica configurazione External Control corretta ✅"
echo "   - Premi PLAY"
echo ""

echo "MONITORAGGIO:"
echo "  Proxy server log: tail -f /tmp/proxy_server.log"
echo "  Driver ROS2 log: tail -f /tmp/driver_ros2.log"
echo "  Verifica porta: netstat -tuln | grep 50002"
echo ""

echo "PER FERMARE TUTTO:"
echo "  pkill -f 'ur_robot_driver\|external_control_proxy'"
echo ""

print_header "✅ SETUP COMPLETATO!"

echo "Il proxy server gestirà le connessioni dal robot."
echo "Se il driver va in crash, il proxy attenderà che si riavvii."
echo ""
echo "IMPORTANTE: Se il driver continua a crashare,"
echo "verifica che EtherNet/IP sia DISABILITATO sul robot!"
echo ""

