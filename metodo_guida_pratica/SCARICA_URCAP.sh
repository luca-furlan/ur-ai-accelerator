#!/bin/bash
# Script per scaricare automaticamente l'URCap External Control

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

URCAP_FILE="externalcontrol-1.0.5.urcap"
URCAP_URL="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/${URCAP_FILE}"

echo "================================================================================"
echo "SCARICA URCAP EXTERNAL CONTROL"
echo "================================================================================"
echo

if [ -f "$URCAP_FILE" ]; then
    echo "✅ File URCap già presente: $URCAP_FILE"
    echo "   Dimensione: $(du -h "$URCAP_FILE" | cut -f1)"
    echo "   Path: $(pwd)/$URCAP_FILE"
else
    echo "📥 Scaricamento URCap..."
    
    if command -v wget >/dev/null 2>&1; then
        wget "$URCAP_URL" -O "$URCAP_FILE"
    elif command -v curl >/dev/null 2>&1; then
        curl -L -o "$URCAP_FILE" "$URCAP_URL"
    else
        echo "❌ wget o curl non disponibili. Installa uno dei due."
        exit 1
    fi
    
    if [ -f "$URCAP_FILE" ]; then
        echo "✅ URCap scaricato con successo!"
        echo "   File: $URCAP_FILE"
        echo "   Dimensione: $(du -h "$URCAP_FILE" | cut -f1)"
        echo "   Path: $(pwd)/$URCAP_FILE"
    else
        echo "❌ Errore durante lo scaricamento"
        exit 1
    fi
fi

echo
echo "================================================================================"
echo "PROSSIMI PASSI"
echo "================================================================================"
echo
echo "1. Copia il file '$URCAP_FILE' su una chiavetta USB"
echo "2. Inserisci la USB nel Teach Pendant"
echo "3. Sul Teach Pendant: Installation → URCaps → '+' → Seleziona file"
echo "4. Riavvia il robot quando richiesto"
echo



