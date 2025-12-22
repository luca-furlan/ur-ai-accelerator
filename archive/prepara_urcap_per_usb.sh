#!/bin/bash
# Script per preparare URCap External Control per installazione via USB

echo "=========================================="
echo "PREPARAZIONE URCAP PER USB"
echo "=========================================="
echo ""

URCAP_VERSION="1.0.5"
URCAP_FILE="externalcontrol-${URCAP_VERSION}.urcap"
URCAP_URL="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/${URCAP_FILE}"

# 1. Scarica URCap
echo "1. Download External Control URCap..."
if [ ! -f "$URCAP_FILE" ]; then
    echo "   Download da GitHub..."
    wget -q --show-progress "$URCAP_URL" -O "$URCAP_FILE" || {
        echo "   ❌ Download fallito!"
        echo "   URL: $URCAP_URL"
        exit 1
    }
    echo "   ✅ URCap scaricato"
else
    echo "   ✅ URCap già presente"
fi

# Verifica file
if [ ! -f "$URCAP_FILE" ]; then
    echo "   ❌ File non trovato!"
    exit 1
fi

FILE_SIZE=$(stat -c%s "$URCAP_FILE" 2>/dev/null || stat -f%z "$URCAP_FILE" 2>/dev/null)
echo "   Dimensione: $FILE_SIZE bytes"

# 2. Verifica integrità
echo ""
echo "2. Verifica file..."
if file "$URCAP_FILE" | grep -q "Zip"; then
    echo "   ✅ File valido (ZIP/URCap)"
else
    echo "   ⚠️  File potrebbe non essere valido"
fi

# 3. Istruzioni
echo ""
echo "=========================================="
echo "✅ FILE PRONTO PER USB!"
echo "=========================================="
echo ""
echo "PROSSIMI PASSI:"
echo ""
echo "1. Copia questo file su una chiavetta USB:"
echo "   $URCAP_FILE"
echo ""
echo "2. Inserisci la USB nel Teach Pendant"
echo ""
echo "3. Sul Teach Pendant:"
echo "   - Vai su: ☰ Menu → Settings → System → URCaps"
echo "   - Clicca '+' (aggiungi)"
echo "   - Naviga alla USB"
echo "   - Seleziona: $URCAP_FILE"
echo "   - Riavvia il robot quando richiesto"
echo ""
echo "4. Dopo il riavvio:"
echo "   - Vai su: Program → New Program"
echo "   - Nel menu URCaps, trova 'External Control'"
echo "   - Trascina il nodo nel programma"
echo "   - Configura:"
echo "     * Host IP: 192.168.10.191"
echo "     * Port: 50002"
echo "   - Salva il programma"
echo "   - Premi PLAY"
echo ""
echo "=========================================="
echo ""
echo "File pronto: $(pwd)/$URCAP_FILE"
echo ""











