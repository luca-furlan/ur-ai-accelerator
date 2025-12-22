#!/bin/bash

# Script di diagnostica specifica per porta 50002
# Basato su Issue #31 e #37 di GitHub
# Verifica perché il Teach Pendant non si connette a 192.168.10.191:50002

PC_IP="192.168.10.191"
ROBOT_IP="192.168.10.194"
PORT="50002"

echo "=================================================================================="
echo "🔍 DIAGNOSTICA PORTA 50002 - Connection Refused dal Teach Pendant"
echo "=================================================================================="
echo ""
echo "Basato su Issue #31 e #37 di GitHub"
echo "PC IP: $PC_IP"
echo "Robot IP: $ROBOT_IP"
echo "Porta: $PORT"
echo ""
echo "=================================================================================="
echo ""

# Contatori
PROBLEMS=0
FIXES=0

# Funzione per stampare risultati
print_status() {
    local status=$1
    local message=$2
    case $status in
        "OK")
            echo "✅ $message"
            ;;
        "ERROR")
            echo "❌ $message"
            ((PROBLEMS++))
            ;;
        "WARN")
            echo "⚠️  $message"
            ((PROBLEMS++))
            ;;
        "FIX")
            echo "🔧 $message"
            ((FIXES++))
            ;;
    esac
}

# ================================================================================
# VERIFICA 1: MACCHINA IN ASCOLTO SULLA PORTA 50002
# ================================================================================
echo "1. VERIFICA MACCHINA IN ASCOLTO SULLA PORTA $PORT"
echo "-------------------------------------------------"

if netstat -tuln | grep -q ":$PORT "; then
    print_status "OK" "Macchina remota IN ASCOLTO sulla porta $PORT"
    echo ""
    echo "   Dettagli:"
    netstat -tuln | grep ":$PORT " | while read line; do
        echo "   $line"
    done
else
    print_status "ERROR" "Macchina remota NON in ascolto sulla porta $PORT"
    echo ""
    echo "   💡 SOLUZIONE (Issue #37):"
    echo "   1. Avvia driver ROS2 PRIMA:"
    echo "      cd ~/MekoAiAccelerator/metodo_guida_pratica"
    echo "      ./START_RAPIDO.sh"
    echo ""
    echo "   2. Il driver ROS2 mette automaticamente la macchina in ascolto sulla 50002"
fi
echo ""

# ================================================================================
# VERIFICA 2: DRIVER ROS2 IN ESECUZIONE
# ================================================================================
echo "2. VERIFICA DRIVER ROS2 IN ESECUZIONE"
echo "--------------------------------------"

if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    print_status "OK" "Driver ROS2 in esecuzione"
    echo ""
    echo "   Processi attivi:"
    ps aux | grep -E "ur_robot_driver|ur_control.launch" | grep -v grep | head -5 | while read line; do
        echo "   $line"
    done
else
    print_status "ERROR" "Driver ROS2 NON in esecuzione"
    echo ""
    echo "   💡 SOLUZIONE:"
    echo "   Avvia driver ROS2 PRIMA di avviare programma sul robot:"
    echo "   cd ~/MekoAiAccelerator/metodo_guida_pratica"
    echo "   ./START_RAPIDO.sh"
fi
echo ""

# ================================================================================
# VERIFICA 3: FIREWALL BLOCCA PORTA 50002
# ================================================================================
echo "3. VERIFICA FIREWALL BLOCCA PORTA $PORT"
echo "----------------------------------------"

# Verifica UFW
if command -v ufw > /dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | head -1)
    if echo "$UFW_STATUS" | grep -q "inactive\|Status: inactive"; then
        print_status "OK" "UFW firewall non attivo (non blocca)"
    else
        print_status "WARN" "UFW firewall attivo"
        echo ""
        echo "   Verifica porta $PORT:"
        if sudo ufw status | grep -q "$PORT"; then
            print_status "OK" "Porta $PORT già aperta in UFW"
        else
            print_status "ERROR" "Porta $PORT NON aperta in UFW (BLOCCATA!)"
            echo ""
            echo "   💡 SOLUZIONE (Issue #37):"
            echo "   Apri porta nel firewall:"
            echo "   sudo ufw allow $PORT"
            echo "   sudo ufw reload"
            echo ""
            echo "   Vuoi che lo script apra la porta ora? (richiede sudo)"
            read -p "   [y/N]: " answer
            if [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then
                if sudo ufw allow $PORT 2>&1; then
                    sudo ufw reload 2>&1
                    print_status "FIX" "Porta $PORT aperta nel firewall"
                else
                    print_status "ERROR" "Impossibile aprire porta (serve sudo)"
                fi
            fi
        fi
    fi
else
    print_status "WARN" "UFW non installato"
    echo ""
    echo "   Verifica iptables:"
    if command -v iptables > /dev/null 2>&1; then
        if sudo iptables -L -n | grep -q "$PORT"; then
            print_status "OK" "Porta $PORT trovata in iptables"
        else
            print_status "WARN" "Porta $PORT NON trovata in iptables"
            echo ""
            echo "   💡 Se firewall è attivo, apri porta:"
            echo "   sudo iptables -A INPUT -p tcp --dport $PORT -j ACCEPT"
            echo "   sudo iptables-save"
        fi
    fi
fi
echo ""

# ================================================================================
# VERIFICA 4: TEST ACCESSIBILITÀ PORTA DAL ROBOT
# ================================================================================
echo "4. TEST ACCESSIBILITÀ PORTA DAL ROBOT"
echo "--------------------------------------"

echo "Test se robot può raggiungere porta $PORT su $PC_IP:"
if timeout 3 nc -zv "$PC_IP" "$PORT" 2>&1 | grep -q "succeeded\|open"; then
    print_status "OK" "Robot può raggiungere porta $PORT su $PC_IP"
else
    print_status "ERROR" "Robot NON può raggiungere porta $PORT su $PC_IP"
    echo ""
    echo "   Possibili cause:"
    echo "   1. Macchina non in ascolto sulla 50002"
    echo "   2. Firewall blocca porta"
    echo "   3. Driver ROS2 non avviato"
fi
echo ""

# ================================================================================
# VERIFICA 5: PROCESSI CHE OCCUPANO PORTA 50002
# ================================================================================
echo "5. VERIFICA CONFLITTI PORTA $PORT"
echo "---------------------------------"

if command -v lsof > /dev/null 2>&1; then
    PROCESSES=$(sudo lsof -i :$PORT 2>/dev/null)
    if [ -z "$PROCESSES" ]; then
        print_status "OK" "Nessun processo occupa porta $PORT"
    else
        print_status "WARN" "Processi che usano porta $PORT:"
        echo "$PROCESSES" | while read line; do
            echo "   $line"
        done
        echo ""
        echo "   💡 Se non è il driver ROS2, chiudi altri processi"
    fi
else
    print_status "WARN" "lsof non installato"
    echo "   Verifica manualmente: sudo netstat -tulpn | grep $PORT"
fi
echo ""

# ================================================================================
# VERIFICA 6: CONFIGURAZIONE CORRETTA (ISTRUZIONI)
# ================================================================================
echo "6. VERIFICA CONFIGURAZIONE TEACH PENDANT"
echo "----------------------------------------"
echo ""
echo "⚠️  VERIFICA SUL TEACH PENDANT (Issue #31):"
echo ""
echo "1. EtherNet/IP DISABILITATO (IMPORTANTE!):"
echo "   - Vai su: Installation → Fieldbus"
echo "   - EtherNet/IP deve essere DISABILITATO ❌"
echo "   - PROFINET deve essere DISABILITATO ❌"
echo "   - Solo Ethernet normale può essere abilitato ✅"
echo ""
echo "2. Remote Control abilitato:"
echo "   - Vai su: Settings → System → Remote Control"
echo "   - Deve essere 'Enabled'"
echo ""
echo "3. Configurazione External Control:"
echo "   - Host IP: $PC_IP"
echo "   - Port: $PORT"
echo "   - Host Name: (vuoto)"
echo ""
echo "4. Ordine corretto:"
echo "   - PRIMA: Avvia driver ROS2 sulla macchina remota"
echo "   - POI: Avvia programma sul robot (PLAY)"
echo ""

# ================================================================================
# VERIFICA 7: CONNETTIVITÀ DI RETE
# ================================================================================
echo "7. VERIFICA CONNETTIVITÀ DI RETE"
echo "--------------------------------"

echo "Test ping da PC a Robot:"
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    print_status "OK" "Robot raggiungibile via ping"
else
    print_status "ERROR" "Robot NON raggiungibile via ping"
fi

echo ""
echo "Test ping da Robot a PC:"
if timeout 3 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=2 "$ROBOT_IP" "ping -c 2 -W 2 $PC_IP" 2>&1 | grep -q "2 received\|2 packets received"; then
    print_status "OK" "PC raggiungibile dal robot"
else
    print_status "WARN" "Impossibile verificare ping dal robot (SSH non disponibile)"
fi
echo ""

# ================================================================================
# VERIFICA 8: LOG DRIVER ROS2
# ================================================================================
echo "8. VERIFICA LOG DRIVER ROS2"
echo "---------------------------"

if pgrep -f "ur_robot_driver\|ur_control.launch" > /dev/null; then
    echo "Ultimi messaggi driver (se disponibili):"
    if [ -f /tmp/driver.log ]; then
        tail -10 /tmp/driver.log 2>/dev/null | grep -E "ERROR|WARN|INFO.*50002|INFO.*connection" | tail -5 || echo "   Nessun messaggio rilevante"
    else
        echo "   Log non trovato in /tmp/driver.log"
    fi
else
    echo "   Driver ROS2 non in esecuzione, nessun log disponibile"
fi
echo ""

# ================================================================================
# RIEPILOGO E SOLUZIONI
# ================================================================================
echo "=================================================================================="
echo "📋 RIEPILOGO DIAGNOSTICA"
echo "=================================================================================="
echo ""

if [ $PROBLEMS -eq 0 ]; then
    echo "✅ Nessun problema trovato!"
    echo ""
    echo "Se il Teach Pendant ancora dice 'Connection refused':"
    echo "1. Verifica EtherNet/IP è DISABILITATO sul robot (Issue #31)"
    echo "2. Verifica configurazione External Control: Host IP = $PC_IP, Port = $PORT"
    echo "3. Avvia driver ROS2 PRIMA, POI programma sul robot"
else
    echo "⚠️  Problemi trovati: $PROBLEMS"
    echo "🔧 Fix applicati: $FIXES"
    echo ""
    echo "SOLUZIONI BASATE SU ISSUE GITHUB:"
    echo ""
    echo "Issue #31 - EtherNet/IP abilitato:"
    echo "  → DISABILITA EtherNet/IP sul Teach Pendant: Installation → Fieldbus"
    echo ""
    echo "Issue #37 - Connection refused:"
    echo "  → Verifica macchina in ascolto: netstat -tuln | grep 50002"
    echo "  → Verifica firewall: sudo ufw allow 50002"
    echo "  → Avvia driver ROS2 PRIMA del programma sul robot"
    echo ""
fi

echo ""
echo "=================================================================================="
echo "🎯 PROCEDURA CORRETTA (ORDINE IMPORTANTE!)"
echo "=================================================================================="
echo ""
echo "1. ✅ Verifica EtherNet/IP DISABILITATO sul robot"
echo "2. ✅ Verifica Remote Control abilitato"
echo "3. ✅ Configura External Control: Host IP = $PC_IP, Port = $PORT"
echo "4. ✅ Avvia driver ROS2 PRIMA: ./START_RAPIDO.sh"
echo "5. ✅ Verifica macchina in ascolto: netstat -tuln | grep 50002"
echo "6. ✅ Avvia programma sul robot (PLAY)"
echo ""
echo "=================================================================================="








