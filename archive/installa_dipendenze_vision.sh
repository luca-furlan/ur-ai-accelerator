#!/bin/bash

# ================================================================================
# INSTALLAZIONE AUTOMATICA DIPENDENZE VISION SYSTEM
# Installa tutto quello che serve per il sistema vision + robot
# ================================================================================

set -e

echo "=================================================================================="
echo "🔧 INSTALLAZIONE DIPENDENZE VISION SYSTEM"
echo "=================================================================================="
echo ""

# Colori
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Funzioni helper
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}→ $1${NC}"
}

# ========================================
# 1. Verifica sistema
# ========================================
echo "[1/6] Verifica sistema..."
echo ""

# Verifica Ubuntu
if [ -f /etc/os-release ]; then
    . /etc/os-release
    print_success "Sistema: $NAME $VERSION"
else
    print_error "Sistema operativo non riconosciuto"
    exit 1
fi

# Verifica utente
print_info "Utente: $(whoami)"
print_info "Directory: $(pwd)"
echo ""

# ========================================
# 2. Source ROS2
# ========================================
echo "[2/6] Configurazione ROS2..."
echo ""

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    print_success "ROS2 Humble trovato"
else
    print_error "ROS2 Humble non trovato!"
    echo "Installa con: sudo apt install ros-humble-desktop"
    exit 1
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    print_success "ROS2 workspace trovato"
else
    print_warning "ROS2 workspace non trovato (~/ros2_ws/install/setup.bash)"
    echo "  Verrà creato se necessario"
fi

echo ""

# ========================================
# 3. Installa pacchetti Python
# ========================================
echo "[3/6] Installazione pacchetti Python..."
echo ""

print_info "Aggiornamento pip..."
pip3 install --upgrade pip --user -q

# Array di pacchetti da installare
PYTHON_PACKAGES=(
    "ultralytics"      # YOLOv8
    "opencv-python"    # OpenCV
    "numpy"            # NumPy
    "flask"            # Web framework
    "requests"         # HTTP library
    "Pillow"           # Image processing
)

for package in "${PYTHON_PACKAGES[@]}"; do
    print_info "Installazione $package..."
    if pip3 install "$package" --user -q; then
        print_success "$package installato"
    else
        print_error "Errore installazione $package"
    fi
done

echo ""

# ========================================
# 4. Installa pacchetti ROS2
# ========================================
echo "[4/6] Installazione pacchetti ROS2..."
echo ""

# cv_bridge
print_info "Verifica cv_bridge..."
if ros2 pkg list | grep -q cv_bridge; then
    print_success "cv_bridge già installato"
else
    print_info "Installazione cv_bridge..."
    if sudo apt install -y ros-humble-cv-bridge; then
        print_success "cv_bridge installato"
    else
        print_error "Errore installazione cv_bridge"
    fi
fi

# vision_msgs
print_info "Verifica vision_msgs..."
if ros2 pkg list | grep -q vision_msgs; then
    print_success "vision_msgs già installato"
else
    print_info "Installazione vision_msgs..."
    if sudo apt install -y ros-humble-vision-msgs; then
        print_success "vision_msgs installato"
    else
        print_error "Errore installazione vision_msgs"
    fi
fi

# image_transport
print_info "Verifica image_transport..."
if ros2 pkg list | grep -q image_transport; then
    print_success "image_transport già installato"
else
    print_info "Installazione image_transport..."
    if sudo apt install -y ros-humble-image-transport; then
        print_success "image_transport installato"
    else
        print_warning "Errore installazione image_transport (opzionale)"
    fi
fi

echo ""

# ========================================
# 5. Installa driver camera Orbecc
# ========================================
echo "[5/6] Configurazione driver camera Orbecc..."
echo ""

if ros2 pkg list | grep -q orbbec_camera; then
    print_success "Driver Orbecc già installato"
else
    print_warning "Driver Orbecc non trovato"
    echo ""
    read -p "Vuoi installare il driver camera Orbecc? (y/n) " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Creazione workspace ROS2..."
        
        # Crea workspace se non esiste
        if [ ! -d ~/ros2_ws ]; then
            mkdir -p ~/ros2_ws/src
            cd ~/ros2_ws
            colcon build
            print_success "Workspace creato"
        fi
        
        cd ~/ros2_ws/src
        
        # Clone repository
        print_info "Clone repository Orbecc..."
        if [ -d OrbbecSDK_ROS2 ]; then
            print_warning "Repository già presente, aggiorno..."
            cd OrbbecSDK_ROS2
            git pull
            cd ..
        else
            if git clone https://github.com/orbbec/OrbbecSDK_ROS2.git; then
                print_success "Repository clonato"
            else
                print_error "Errore clone repository"
            fi
        fi
        
        # Build
        print_info "Build driver Orbecc (potrebbe richiedere alcuni minuti)..."
        cd ~/ros2_ws
        source /opt/ros/humble/setup.bash
        
        if colcon build --packages-select orbbec_camera; then
            print_success "Driver Orbecc compilato"
            source install/setup.bash
            print_success "Driver Orbecc installato"
        else
            print_error "Errore compilazione driver Orbecc"
        fi
    else
        print_warning "Installazione driver Orbecc saltata"
        echo "  Nota: Camera non funzionerà senza driver"
        echo "  Installa manualmente con:"
        echo "    cd ~/ros2_ws/src"
        echo "    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git"
        echo "    cd ~/ros2_ws && colcon build --packages-select orbbec_camera"
    fi
fi

echo ""

# ========================================
# 6. Download modello YOLO
# ========================================
echo "[6/6] Download modello YOLOv8..."
echo ""

print_info "Download YOLOv8 nano model..."
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); print('✅ Modello scaricato')" 2>&1 | tail -1

echo ""

# ========================================
# Verifica finale
# ========================================
echo "=================================================================================="
echo "✅ VERIFICA INSTALLAZIONE"
echo "=================================================================================="
echo ""

# Test Python packages
print_info "Test pacchetti Python..."
echo ""

test_package() {
    local package=$1
    local import_name=${2:-$1}
    
    if python3 -c "import $import_name" 2>/dev/null; then
        VERSION=$(python3 -c "import $import_name; print(getattr($import_name, '__version__', 'OK'))" 2>/dev/null)
        print_success "$package ($VERSION)"
        return 0
    else
        print_error "$package NON installato"
        return 1
    fi
}

test_package "ultralytics" "ultralytics"
test_package "OpenCV" "cv2"
test_package "NumPy" "numpy"
test_package "Flask" "flask"
test_package "Requests" "requests"
test_package "Pillow" "PIL"

echo ""
print_info "Test pacchetti ROS2..."
echo ""

test_ros_package() {
    local package=$1
    
    if ros2 pkg list | grep -q "^$package$"; then
        print_success "$package"
        return 0
    else
        print_error "$package NON installato"
        return 1
    fi
}

test_ros_package "cv_bridge"
test_ros_package "vision_msgs"
test_ros_package "orbbec_camera"

echo ""
print_info "Test import Python ROS2..."
echo ""

# Test cv_bridge
if python3 -c "from cv_bridge import CvBridge; print('OK')" 2>/dev/null | grep -q OK; then
    print_success "cv_bridge Python OK"
else
    print_error "cv_bridge Python NON funzionante"
fi

# Test YOLO
if python3 -c "from ultralytics import YOLO; m = YOLO('yolov8n.pt'); print('OK')" 2>/dev/null | grep -q OK; then
    print_success "YOLOv8 OK"
else
    print_error "YOLOv8 NON funzionante"
fi

echo ""

# ========================================
# Riepilogo finale
# ========================================
echo "=================================================================================="
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""

# Conta successi
SUCCESS_COUNT=0
TOTAL_COUNT=8

# Python packages
python3 -c "import ultralytics" 2>/dev/null && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
python3 -c "import cv2" 2>/dev/null && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
python3 -c "import flask" 2>/dev/null && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))

# ROS2 packages
ros2 pkg list | grep -q cv_bridge && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
ros2 pkg list | grep -q vision_msgs && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))

# Python ROS2 integration
python3 -c "from cv_bridge import CvBridge" 2>/dev/null && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))

# YOLO
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" 2>/dev/null && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))

# Camera
ros2 pkg list | grep -q orbbec_camera && SUCCESS_COUNT=$((SUCCESS_COUNT + 1))

echo "Pacchetti installati: $SUCCESS_COUNT/$TOTAL_COUNT"
echo ""

if [ $SUCCESS_COUNT -eq $TOTAL_COUNT ]; then
    print_success "INSTALLAZIONE COMPLETATA CON SUCCESSO!"
    echo ""
    echo "Prossimi passi:"
    echo "  1. Collega camera Orbecc via USB"
    echo "  2. Test sistema: python3 test_vision_system.py"
    echo "  3. Avvia: ./avvia_web_interface_con_vision.sh"
elif [ $SUCCESS_COUNT -ge 6 ]; then
    print_warning "INSTALLAZIONE COMPLETATA CON AVVISI"
    echo ""
    echo "Alcuni pacchetti opzionali non installati."
    echo "Il sistema dovrebbe funzionare comunque."
    echo ""
    echo "Test sistema: python3 test_vision_system.py"
else
    print_error "INSTALLAZIONE INCOMPLETA"
    echo ""
    echo "Alcuni pacchetti essenziali mancano."
    echo "Risolvi gli errori e riesegui lo script."
fi

echo ""
echo "=================================================================================="




