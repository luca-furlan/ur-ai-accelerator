#!/bin/bash

# Script per verificare se EtherNet/IP è abilitato sul robot
# ATTENZIONE: EtherNet/IP non può essere verificato via SSH
# Questo script fornisce istruzioni chiare

echo "=================================================================================="
echo "🔍 VERIFICA ETHERNET/IP SUL ROBOT"
echo "=================================================================================="
echo ""
echo "⚠️  IMPORTANTE: EtherNet/IP NON può essere verificato via SSH"
echo "   Devi controllare manualmente sul Teach Pendant"
echo ""
echo "=================================================================================="
echo ""

# Test connessione robot
ROBOT_IP="192.168.10.194"
echo "1. Test connessione robot..."
if ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile"
else
    echo "   ❌ Robot NON raggiungibile"
    exit 1
fi
echo ""

# Verifica porte che potrebbero indicare EtherNet/IP
echo "2. Verifica porte che potrebbero indicare EtherNet/IP attivo..."
echo ""

# EtherNet/IP usa tipicamente porta 44818
if timeout 3 nc -zv "$ROBOT_IP" 44818 2>&1 | grep -q "succeeded\|open"; then
    echo "   ⚠️  Porta 44818 (EtherNet/IP) APERTA sul robot"
    echo "   Questo potrebbe indicare EtherNet/IP abilitato!"
else
    echo "   ✅ Porta 44818 (EtherNet/IP) chiusa"
fi
echo ""

# Verifica RTDE disponibile
echo "3. Verifica RTDE disponibile..."
if timeout 3 nc -zv "$ROBOT_IP" 30004 2>&1 | grep -q "succeeded\|open"; then
    echo "   ✅ Porta 30004 (RTDE) aperta"
    echo "   RTDE è disponibile (buon segno)"
else
    echo "   ⚠️  Porta 30004 (RTDE) chiusa"
    echo "   RTDE potrebbe essere occupato da EtherNet/IP!"
fi
echo ""

# Istruzioni chiare
echo "=================================================================================="
echo "📋 ISTRUZIONI PER VERIFICARE ETHERNET/IP"
echo "=================================================================================="
echo ""
echo "Sul Teach Pendant:"
echo ""
echo "1. Vai su: Installation → Fieldbus"
echo ""
echo "2. Cerca nella lista:"
echo "   - EtherNet/IP"
echo "   - PROFINET"
echo ""
echo "3. Verifica stato:"
echo "   - EtherNet/IP deve essere DISABILITATO ❌"
echo "   - PROFINET deve essere DISABILITATO ❌"
echo "   - Solo Ethernet normale può essere abilitato ✅"
echo ""
echo "4. Se EtherNet/IP è abilitato:"
echo "   - Tocca EtherNet/IP"
echo "   - DISABILITALO"
echo "   - Salva"
echo "   - RIAVVIA robot (power cycle)"
echo ""
echo "=================================================================================="
echo ""
echo "SE ETHERNET/IP È ABILITATO:"
echo "  → Il driver ROS2 va in crash"
echo "  → Il PC non si mette in ascolto sulla 50002"
echo "  → Il robot non può connettersi"
echo "  → Il programma va in errore"
echo ""
echo "QUESTO È IL PROBLEMA PRINCIPALE (Issue #31 GitHub)!"
echo ""








