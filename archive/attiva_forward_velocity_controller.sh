#!/bin/bash
# Script per attivare forward_velocity_controller

echo "=================================================================================="
echo "🔧 ATTIVAZIONE FORWARD_VELOCITY_CONTROLLER"
echo "=================================================================================="
echo ""

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Verifica controller disponibili..."
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers 2>&1 | grep -A 5 "forward_velocity_controller" || echo "   Controller non trovato nella lista"

echo ""
echo "2. Verifica controller attivi..."
ACTIVE=$(ros2 param get /controller_manager active_controllers 2>&1)
echo "$ACTIVE" | grep -E "forward_velocity|scaled_joint" || echo "   Nessun controller trovato"

echo ""
echo "3. Tentativo di attivazione forward_velocity_controller..."
echo "   (Questo potrebbe non funzionare se il controller non è configurato)"

# Prova a switchare controller
# Prima ferma eventuali controller attivi
# Poi attiva forward_velocity_controller

# Verifica se esiste il servizio switch_controllers
if ros2 service list | grep -q "switch_controllers"; then
    echo "   ✅ Servizio switch_controllers disponibile"
    echo ""
    echo "   ⚠️  ATTENZIONE: Questo potrebbe fermare altri controller!"
    echo "   Vuoi procedere? (s/n)"
    read -p "   " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Ss]$ ]]; then
        # Attiva forward_velocity_controller
        # Formato: ros2 service call /controller_manager/switch_controllers controller_manager_msgs/srv/SwitchControllers "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: [], strictness: 1}"
        echo "   Tentativo di attivazione..."
        ros2 service call /controller_manager/switch_controllers controller_manager_msgs/srv/SwitchControllers "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: [], strictness: 1}" 2>&1
        
        sleep 2
        
        echo ""
        echo "4. Verifica controller attivo..."
        ros2 param get /controller_manager active_controllers 2>&1 | grep forward_velocity && echo "   ✅ Controller attivato!" || echo "   ❌ Controller NON attivato"
    else
        echo "   Operazione annullata"
    fi
else
    echo "   ⚠️  Servizio switch_controllers non disponibile"
    echo "   💡 Il controller potrebbe essere già attivo o configurato diversamente"
fi

echo ""
echo "=================================================================================="
echo "📋 PROSSIMI PASSI"
echo "=================================================================================="
echo ""
echo "Se il controller è attivo, prova:"
echo "  ros2 topic pub -r 10 /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}'"
echo ""
echo "Premi CTRL+C dopo 2-3 secondi per fermare"
echo ""







