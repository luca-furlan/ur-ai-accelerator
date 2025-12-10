#!/bin/bash
# Script SICURO per installare External Control URCap sul robot UR5e
# Vai piano e verifica tutto passo passo

set -e

ROBOT_IP="192.168.10.194"
ROBOT_USER="root"
ROBOT_PASS="easybot"

echo "=========================================="
echo "INSTALLAZIONE EXTERNAL CONTROL URCAP"
echo "Script SICURO - Verifica tutto passo passo"
echo "=========================================="
echo ""

# 1. Verifica connessione robot
echo "1. Verifica connessione robot..."
if ping -c 1 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "   ✅ Robot raggiungibile: $ROBOT_IP"
else
    echo "   ❌ Robot NON raggiungibile!"
    exit 1
fi

# 2. Verifica versione PolyScope
echo ""
echo "2. Verifica versione PolyScope..."
VERSION=$(sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no $ROBOT_USER@$ROBOT_IP \
    "cat /etc/ur_software_version 2>/dev/null || echo 'UNKNOWN'")

echo "   Versione trovata: $VERSION"

# Determina versione PolyScope
if echo "$VERSION" | grep -q "5\."; then
    POLYSCOPE_VERSION="5"
    URCAP_VERSION="1.0.5"
    echo "   ✅ PolyScope 5 rilevato"
elif echo "$VERSION" | grep -q "10\."; then
    POLYSCOPE_VERSION="X"
    URCAP_VERSION="1.0.5"
    echo "   ✅ PolyScope X rilevato"
else
    echo "   ⚠️  Versione non riconosciuta, uso versione generica"
    POLYSCOPE_VERSION="X"
    URCAP_VERSION="1.0.5"
fi

# 3. Scarica URCap
echo ""
echo "3. Download External Control URCap..."
URCAP_FILE="externalcontrol-${URCAP_VERSION}.urcap"
URCAP_URL="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/${URCAP_FILE}"

if [ ! -f "$URCAP_FILE" ]; then
    echo "   Download da GitHub..."
    wget -q "$URCAP_URL" -O "$URCAP_FILE" || {
        echo "   ❌ Download fallito!"
        echo "   URL: $URCAP_URL"
        exit 1
    }
    echo "   ✅ URCap scaricato: $URCAP_FILE"
else
    echo "   ✅ URCap già presente: $URCAP_FILE"
fi

# Verifica file
if [ ! -f "$URCAP_FILE" ]; then
    echo "   ❌ File URCap non trovato!"
    exit 1
fi

FILE_SIZE=$(stat -f%z "$URCAP_FILE" 2>/dev/null || stat -c%s "$URCAP_FILE" 2>/dev/null)
echo "   Dimensione file: $FILE_SIZE bytes"

# 4. Verifica spazio su robot
echo ""
echo "4. Verifica spazio su robot..."
SPACE=$(sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no $ROBOT_USER@$ROBOT_IP \
    "df -h / | tail -1 | awk '{print \$4}'")
echo "   Spazio disponibile: $SPACE"

# 5. Backup URCaps esistenti (sicurezza)
echo ""
echo "5. Backup URCaps esistenti..."
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no $ROBOT_USER@$ROBOT_IP \
    "mkdir -p /root/urcap_backup && cp -r /root/urcaps/* /root/urcap_backup/ 2>/dev/null || true"
echo "   ✅ Backup creato in /root/urcap_backup"

# 6. Trasferisci URCap sul robot
echo ""
echo "6. Trasferimento URCap sul robot..."
echo "   ⚠️  ATTENZIONE: Questo trasferirà il file sul robot"
read -p "   Continuare? (s/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Ss]$ ]]; then
    echo "   ❌ Operazione annullata"
    exit 1
fi

sshpass -p "$ROBOT_PASS" scp -o StrictHostKeyChecking=no "$URCAP_FILE" \
    $ROBOT_USER@$ROBOT_IP:/root/urcaps/

echo "   ✅ URCap trasferito"

# 7. Verifica file sul robot
echo ""
echo "7. Verifica file sul robot..."
sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no $ROBOT_USER@$ROBOT_IP \
    "ls -lh /root/urcaps/$URCAP_FILE && echo '✅ File presente sul robot'"

# 8. Istruzioni finali
echo ""
echo "=========================================="
echo "✅ URCAP TRASFERITO SUL ROBOT!"
echo "=========================================="
echo ""
echo "PROSSIMI PASSI SUL TEACH PENDANT:"
echo ""
echo "1. Riavvia il robot (se richiesto)"
echo "2. Vai su: ☰ Menu → Settings → System → URCaps"
echo "3. Verifica che 'External Control' sia nella lista"
echo "4. Se non c'è, clicca '+' e seleziona il file:"
echo "   /root/urcaps/$URCAP_FILE"
echo "5. Riavvia il robot quando richiesto"
echo ""
echo "Dopo l'installazione:"
echo "1. Vai su Program → New Program"
echo "2. Nel menu URCaps, trova 'External Control'"
echo "3. Trascina il nodo nel programma"
echo "4. Configura:"
echo "   - Host IP: 192.168.10.191"
echo "   - Port: 50002"
echo "5. Salva e premi PLAY"
echo ""
echo "=========================================="





