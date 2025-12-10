#!/bin/bash

# Script completo per verificare e sistemare problemi External Control URCap
# Basato su Issue #31 e #37 di GitHub

# set -e  # Disabilitato per permettere continuazione anche con errori

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROBOT_IP="192.168.10.194"
PC_IP="192.168.10.191"
PORTS=(50001 50002 50003 50004)

echo "=================================================================================="
echo "🔧 FIX COMPLETO AUTOMATICO - External Control URCap"
echo "=================================================================================="
echo ""
echo "Basato su Issue #31 e #37 di GitHub"
echo "Robot IP: $ROBOT_IP"
echo "PC IP: $PC_IP"
echo ""
echo "=================================================================================="
echo ""

# Contatori per riepilogo
FIXED=0
WARNINGS=0
ERRORS=0

# Funzione per stampare risultati
print_result() {
    local status=$1
    local message=$2
    case $status in
        "OK")
            echo "✅ $message"
            ;;
        "WARN")
            echo "⚠️  $message"
            ((WARNINGS++))
            ;;
        "ERROR")
            echo "❌ $message"
            ((ERRORS++))
            ;;
        "FIXED")
            echo "🔧 $message"
            ((FIXED++))
            ;;
    esac
}

# ================================================================================
# PASSO 1: VERIFICA CONNETTIVITÀ DI RETE
# ================================================================================
echo "1. VERIFICA CONNETTIVITÀ DI RETE"
echo "--------------------------------"
if ping -c 2 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    print_result "OK" "Robot raggiungibile: $ROBOT_IP"
else
    print_result "ERROR" "Robot NON raggiungibile: $ROBOT_IP"
    echo "   Verifica connessione di rete fisica"
fi
echo ""

# ================================================================================
# PASSO 2: VERIFICA E SISTEMA FIREWALL
# ================================================================================
echo "2. VERIFICA E SISTEMA FIREWALL"
echo "------------------------------"

# Verifica UFW
if command -v ufw > /dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | head -1)
    if echo "$UFW_STATUS" | grep -q "inactive\|Status: inactive"; then
        print_result "OK" "UFW firewall non attivo"
    else
        print_result "WARN" "UFW firewall attivo"
        
        # Verifica e apre porte
        for PORT in "${PORTS[@]}"; do
            if sudo ufw status | grep -q "$PORT"; then
                print_result "OK" "Porta $PORT già aperta"
            else
                echo "   Aprendo porta $PORT..."
                if sudo ufw allow $PORT > /dev/null 2>&1; then
                    print_result "FIXED" "Porta $PORT aperta nel firewall"
                else
                    print_result "ERROR" "Impossibile aprire porta $PORT (serve sudo)"
                fi
            fi
        done
        
        # Ricarica firewall
        sudo ufw reload > /dev/null 2>&1
    fi
else
    print_result "WARN" "UFW non installato"
    
    # Prova a installare UFW
    echo ""
    echo "   🔧 Tentativo di installare UFW..."
    if command -v apt-get > /dev/null 2>&1; then
        echo "   Installazione UFW (richiede sudo senza password)..."
        echo ""
        
        # Verifica se sudo funziona senza password
        if sudo -n true 2>/dev/null; then
            # Sudo funziona senza password, procedi con installazione
            if sudo apt-get update 2>&1 | tee /tmp/ufw_install.log && sudo apt-get install -y ufw 2>&1 | tee -a /tmp/ufw_install.log; then
                print_result "FIXED" "UFW installato con successo"
                echo ""
                
                # Configura UFW
                echo "   🔧 Configurando UFW..."
                for PORT in "${PORTS[@]}"; do
                    if sudo ufw allow $PORT 2>&1; then
                        print_result "FIXED" "Porta $PORT aperta in UFW"
                    else
                        print_result "ERROR" "Impossibile aprire porta $PORT"
                    fi
                done
                echo ""
                
                # Abilita UFW
                echo "   🔧 Abilitando UFW..."
                if echo "y" | sudo ufw enable 2>&1; then
                    print_result "FIXED" "UFW abilitato e configurato"
                else
                    print_result "WARN" "UFW installato ma non abilitato (potrebbe essere già attivo)"
                fi
                echo ""
            else
                print_result "ERROR" "Impossibile installare UFW"
                echo "   Verifica log: cat /tmp/ufw_install.log"
                echo "   Installa manualmente: sudo apt-get install -y ufw"
                echo ""
            fi
        else
            print_result "WARN" "Sudo richiede password (installazione automatica non possibile)"
            echo ""
            echo "   ⚠️  INSTALLAZIONE MANUALE RICHIESTA:"
            echo "   1. Esegui: sudo apt-get update"
            echo "   2. Esegui: sudo apt-get install -y ufw"
            echo "   3. Esegui: sudo ufw allow 50001"
            echo "   4. Esegui: sudo ufw allow 50002"
            echo "   5. Esegui: sudo ufw allow 50003"
            echo "   6. Esegui: sudo ufw allow 50004"
            echo "   7. Esegui: echo 'y' | sudo ufw enable"
            echo ""
            echo "   OPPURE esegui questo comando completo:"
            echo "   sudo apt-get update && sudo apt-get install -y ufw && sudo ufw allow 50001 && sudo ufw allow 50002 && sudo ufw allow 50003 && sudo ufw allow 50004 && echo 'y' | sudo ufw enable"
            echo ""
        fi
    else
        print_result "WARN" "Sistema non basato su apt-get"
        echo "   Installa UFW manualmente con il gestore pacchetti del tuo sistema"
        echo ""
    fi
    
    # Verifica altri firewall
    echo ""
    echo "   Verificando altri firewall..."
    
    # Verifica iptables
    if command -v iptables > /dev/null 2>&1; then
        IPTABLES_RULES=$(sudo iptables -L -n 2>/dev/null | wc -l)
        if [ "$IPTABLES_RULES" -gt 8 ]; then
            print_result "WARN" "iptables ha regole attive (potrebbe bloccare porte)"
            echo "   Verificando porta 50002..."
            if sudo iptables -L -n | grep -q "50002"; then
                print_result "OK" "Porta 50002 trovata in iptables"
            else
                print_result "WARN" "Porta 50002 NON trovata in iptables"
                echo "   Aggiungendo regola per porta 50002..."
                if sudo iptables -A INPUT -p tcp --dport 50002 -j ACCEPT 2>/dev/null; then
                    print_result "FIXED" "Regola aggiunta per porta 50002"
                    sudo iptables-save > /dev/null 2>&1
                else
                    print_result "ERROR" "Impossibile aggiungere regola (serve sudo)"
                    echo "   Aggiungi manualmente: sudo iptables -A INPUT -p tcp --dport 50002 -j ACCEPT"
                fi
            fi
        else
            print_result "OK" "iptables non ha regole restrittive"
        fi
    fi
    
    # Verifica firewalld (RedHat/CentOS)
    if command -v firewall-cmd > /dev/null 2>&1; then
        if sudo firewall-cmd --state 2>/dev/null | grep -q "running"; then
            print_result "WARN" "firewalld è attivo"
            for PORT in "${PORTS[@]}"; do
                if sudo firewall-cmd --list-ports 2>/dev/null | grep -q "$PORT"; then
                    print_result "OK" "Porta $PORT già aperta in firewalld"
                else
                    echo "   Aprendo porta $PORT in firewalld..."
                    if sudo firewall-cmd --permanent --add-port=$PORT/tcp 2>/dev/null; then
                        print_result "FIXED" "Porta $PORT aperta in firewalld"
                    fi
                fi
            done
            sudo firewall-cmd --reload 2>/dev/null
        fi
    fi
    
    # Verifica se le porte sono effettivamente bloccate
    echo ""
    echo "   Verificando se le porte sono bloccate..."
    for PORT in "${PORTS[@]}"; do
        if timeout 1 bash -c "echo > /dev/tcp/$PC_IP/$PORT" 2>/dev/null; then
            print_result "OK" "Porta $PORT accessibile localmente"
        else
            print_result "WARN" "Porta $PORT potrebbe essere bloccata localmente"
        fi
    done
fi
echo ""

# ================================================================================
# PASSO 3: VERIFICA PORTA 50002 IN ASCOLTO
# ================================================================================
echo "3. VERIFICA MACCHINA IN ASCOLTO SULLA PORTA 50002"
echo "------------------------------------------------"

if netstat -tuln | grep -q ":50002 "; then
    print_result "OK" "Macchina remota in ascolto sulla porta 50002"
    netstat -tuln | grep ":50002 "
else
    print_result "WARN" "Macchina remota NON in ascolto sulla porta 50002"
    echo "   SOLUZIONE: Avvia driver ROS2 PRIMA!"
    echo "   cd ~/MekoAiAccelerator/metodo_guida_pratica"
    echo "   ./START_RAPIDO.sh"
fi
echo ""

# ================================================================================
# PASSO 4: VERIFICA DRIVER ROS2
# ================================================================================
echo "4. VERIFICA DRIVER ROS2"
echo "----------------------"

if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    print_result "OK" "Driver ROS2 in esecuzione"
    echo "   Processi attivi:"
    ps aux | grep "ur_robot_driver\|ur_control.launch" | grep -v grep | head -3 | awk '{print "   - " $11 " (PID: " $2 ")"}'
else
    print_result "WARN" "Driver ROS2 NON in esecuzione"
    echo "   SOLUZIONE: Avvia driver ROS2"
    echo "   cd ~/MekoAiAccelerator/metodo_guida_pratica"
    echo "   ./START_RAPIDO.sh"
fi
echo ""

# ================================================================================
# PASSO 5: VERIFICA ROBOT HA PORTA 50002 APERTA
# ================================================================================
echo "5. VERIFICA ROBOT HA PORTA 50002 APERTA"
echo "---------------------------------------"

if timeout 3 nc -zv $ROBOT_IP 50002 2>&1 | grep -q "succeeded\|open"; then
    print_result "OK" "Robot ha porta 50002 aperta"
else
    print_result "WARN" "Robot NON ha porta 50002 aperta"
    echo "   SOLUZIONE:"
    echo "   1. Sul Teach Pendant: Avvia programma con External Control (PLAY)"
    echo "   2. Verifica programma è in PLAYING"
    echo "   3. Verifica Remote Control è abilitato"
fi
echo ""

# ================================================================================
# PASSO 6: VERIFICA PROCESSI CHE USANO PORTA 50002
# ================================================================================
echo "6. VERIFICA CONFLITTI PORTA 50002"
echo "---------------------------------"

if command -v lsof > /dev/null 2>&1; then
    PROCESSES=$(sudo lsof -i :50002 2>/dev/null)
    if [ -z "$PROCESSES" ]; then
        print_result "OK" "Nessun processo usa porta 50002"
    else
        print_result "WARN" "Processi che usano porta 50002:"
        echo "$PROCESSES" | while read line; do
            echo "   $line"
        done
    fi
else
    print_result "WARN" "lsof non installato, verifica manualmente: sudo netstat -tulpn | grep 50002"
fi
echo ""

# ================================================================================
# PASSO 7: VERIFICA RTDE CONFLICTS
# ================================================================================
echo "7. VERIFICA CONFLITTI RTDE"
echo "-------------------------"

RTDE_PROCESSES=$(ps aux | grep -E "rtde|ur_rtde" | grep -v grep | grep -v "grep" || true)
if [ -z "$RTDE_PROCESSES" ]; then
    print_result "OK" "Nessun altro processo RTDE attivo"
else
    print_result "WARN" "Altri processi RTDE trovati (potrebbero causare conflitti):"
    echo "$RTDE_PROCESSES" | while read line; do
        echo "   $line"
    done
    echo "   SOLUZIONE: Chiudi altri processi RTDE prima di avviare driver ROS2"
fi
echo ""

# ================================================================================
# PASSO 8: ISTRUZIONI PER PROBLEMI SUL TEACH PENDANT
# ================================================================================
echo "8. VERIFICA CONFIGURAZIONE TEACH PENDANT"
echo "---------------------------------------"
echo ""
echo "⚠️  PROBLEMI COMUNI (Issue #31 GitHub):"
echo ""
echo "1. EtherNet/IP ABILITATO (CAUSA PIÙ COMUNE!)"
echo "   Sul Teach Pendant:"
echo "   - Vai su: Installation → Fieldbus"
echo "   - Verifica: EtherNet/IP deve essere DISABILITATO ❌"
echo "   - Verifica: PROFINET deve essere DISABILITATO ❌"
echo "   - Solo Ethernet normale può essere abilitato ✅"
echo ""
echo "2. Remote Control NON abilitato"
echo "   Sul Teach Pendant:"
echo "   - Vai su: Settings → System → Remote Control"
echo "   - Deve essere 'Enabled'"
echo ""
echo "3. Configurazione External Control"
echo "   Sul Teach Pendant, nel nodo External Control:"
echo "   - Host IP: $PC_IP"
echo "   - Porta: 50002"
echo "   - Programma deve essere in PLAYING"
echo ""
echo "=================================================================================="
echo ""

# ================================================================================
# RIEPILOGO FINALE
# ================================================================================
echo "📋 RIEPILOGO"
echo "=================================================================================="
echo ""
echo "Problemi risolti automaticamente: $FIXED"
echo "Avvisi: $WARNINGS"
echo "Errori: $ERRORS"
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo "✅ TUTTO OK! Sistema pronto per External Control"
    echo ""
    echo "Prossimi passi:"
    echo "1. Verifica sul Teach Pendant: EtherNet/IP disabilitato"
    echo "2. Avvia programma sul robot (PLAY)"
    echo "3. Verifica connessione funziona"
elif [ $ERRORS -eq 0 ]; then
    echo "⚠️  Alcuni avvisi trovati. Verifica le istruzioni sopra."
    echo ""
    echo "PROBLEMA PIÙ COMUNE (Issue #31):"
    echo "Sul Teach Pendant: Installation → Fieldbus"
    echo "- DISABILITA EtherNet/IP se è abilitato!"
    echo "- DISABILITA PROFINET se è abilitato!"
else
    echo "❌ Errori trovati. Risolvi gli errori prima di procedere."
fi

echo ""
echo "=================================================================================="
echo ""
echo "Per maggiori dettagli, vedi: ANALISI_ISSUE_GITHUB_COMPLETA.txt"
echo ""

