#!/bin/bash
# Script per fixare il lock di dpkg/apt

echo "================================================================================"
echo "FIX LOCK DPKG/APT"
echo "================================================================================"
echo

# 1. Verifica processi apt in esecuzione
echo "1. Verifica processi apt/dpkg in esecuzione..."
APT_PROCESSES=$(ps aux | grep -E 'apt|dpkg|unattended-upgrade' | grep -v grep)
if [ -n "$APT_PROCESSES" ]; then
    echo "   ⚠️ Processi apt/dpkg trovati:"
    echo "$APT_PROCESSES" | while read line; do
        echo "      $line"
    done
    echo
    echo "   💡 Attendi che finiscano o termina manualmente:"
    echo "      sudo killall apt apt-get dpkg"
else
    echo "   ✅ Nessun processo apt/dpkg in esecuzione"
fi
echo

# 2. Rimuovi lock files
echo "2. Rimozione lock files..."
if [ -f /var/lib/dpkg/lock-frontend ]; then
    echo "   ⚠️ Trovato /var/lib/dpkg/lock-frontend"
    sudo rm -f /var/lib/dpkg/lock-frontend
    echo "   ✅ Rimosso"
else
    echo "   ✅ /var/lib/dpkg/lock-frontend non presente"
fi

if [ -f /var/lib/dpkg/lock ]; then
    echo "   ⚠️ Trovato /var/lib/dpkg/lock"
    sudo rm -f /var/lib/dpkg/lock
    echo "   ✅ Rimosso"
else
    echo "   ✅ /var/lib/dpkg/lock non presente"
fi

if [ -f /var/cache/apt/archives/lock ]; then
    echo "   ⚠️ Trovato /var/cache/apt/archives/lock"
    sudo rm -f /var/cache/apt/archives/lock
    echo "   ✅ Rimosso"
else
    echo "   ✅ /var/cache/apt/archives/lock non presente"
fi

if [ -f /var/lib/apt/lists/lock ]; then
    echo "   ⚠️ Trovato /var/lib/apt/lists/lock"
    sudo rm -f /var/lib/apt/lists/lock
    echo "   ✅ Rimosso"
else
    echo "   ✅ /var/lib/apt/lists/lock non presente"
fi
echo

# 3. Reconfigura dpkg
echo "3. Riconfigurazione dpkg..."
sudo dpkg --configure -a
echo

# 4. Fix eventuali pacchetti rotti
echo "4. Fix pacchetti rotti..."
sudo apt --fix-broken install -y
echo

# 5. Verifica che tutto sia OK
echo "5. Verifica finale..."
if sudo apt update 2>&1 | grep -q "could not get lock"; then
    echo "   ❌ Lock ancora presente"
    echo "   💡 Prova a riavviare la macchina"
else
    echo "   ✅ Lock rimosso, apt funziona correttamente"
fi
echo

echo "================================================================================"
echo "✅ FIX COMPLETATO"
echo "================================================================================"
echo
echo "💡 Se il problema persiste:"
echo "   1. Riavvia la macchina: sudo reboot"
echo "   2. Oppure termina manualmente i processi:"
echo "      sudo killall apt apt-get dpkg"
echo "      sudo rm -f /var/lib/dpkg/lock* /var/cache/apt/archives/lock /var/lib/apt/lists/lock"
echo











