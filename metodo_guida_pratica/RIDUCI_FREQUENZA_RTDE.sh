#!/bin/bash

# Script per ridurre frequenza RTDE e risolvere overflow
# Basato su soluzioni online: ridurre da 500 Hz a 125 Hz

ROBOT_TYPE="ur5e"
RTDE_FREQ="125"  # Ridotta da 500 a 125 Hz

echo "=================================================================================="
echo "🔧 RIDUZIONE FREQUENZA RTDE PER RISOLVERE OVERFLOW"
echo "=================================================================================="
echo ""
echo "Frequenza RTDE: 500 Hz → $RTDE_FREQ Hz"
echo "Questo può permettere a EtherNet/IP e ROS2 di coesistere"
echo ""

cd ~/MekoAiAccelerator/metodo_guida_pratica || exit 1

# Cerca file di configurazione
CONFIG_FILE=$(find ~/ros2_ws -name "ur5e_update_rate.yaml" 2>/dev/null | head -1)

if [ -z "$CONFIG_FILE" ]; then
    echo "⚠️  File ur5e_update_rate.yaml non trovato"
    echo ""
    echo "Crea file di configurazione personalizzato..."
    
    # Crea file temporaneo con frequenza ridotta
    mkdir -p /tmp/ur_config
    cat > /tmp/ur_config/ur5e_low_freq.yaml << EOF
# Configurazione RTDE con frequenza ridotta per evitare overflow
# Utile quando EtherNet/IP è abilitato

ur_robot_driver:
  ur_type: ur5e
  robot_ip: 192.168.10.194
  
  # Frequenza RTDE ridotta per evitare overflow con EtherNet/IP
  rtde_output_recipe_frequency: $RTDE_FREQ
  rtde_input_recipe_frequency: $RTDE_FREQ
  
  # Altre impostazioni
  use_mock_hardware: false
  headless_mode: false
EOF
    
    CONFIG_FILE="/tmp/ur_config/ur5e_low_freq.yaml"
    echo "✅ File creato: $CONFIG_FILE"
else
    echo "✅ File trovato: $CONFIG_FILE"
    echo ""
    echo "⚠️  Modificare file esistente può causare problemi"
    echo "   Creo file personalizzato invece..."
    
    mkdir -p /tmp/ur_config
    cp "$CONFIG_FILE" /tmp/ur_config/ur5e_low_freq.yaml.bak
    CONFIG_FILE="/tmp/ur_config/ur5e_low_freq.yaml"
fi

echo ""
echo "=================================================================================="
echo "📋 CONFIGURAZIONE CREATA"
echo "=================================================================================="
echo ""
echo "File: $CONFIG_FILE"
echo ""
echo "Per usare questa configurazione:"
echo ""
echo "ros2 launch ur_robot_driver ur_control.launch.py \\"
echo "    ur_type:=$ROBOT_TYPE \\"
echo "    robot_ip:=192.168.10.194 \\"
echo "    launch_rviz:=false \\"
echo "    --params-file $CONFIG_FILE"
echo ""
echo "=================================================================================="
echo ""
echo "⚠️  NOTA: Se EtherNet/IP è necessario, prova questa configurazione"
echo "   Se non funziona, EtherNet/IP deve essere disabilitato"
echo ""

