#!/bin/bash
# Script di Pulizia AI Accelerator
# Eseguire su AI Accelerator: bash cleanup_ai_accelerator.sh

echo "=== PULIZIA AI ACCELERATOR ==="
echo ""

# Verifica spazio iniziale
echo "📊 Spazio iniziale:"
df -h / | grep -v Filesystem
echo ""

# 1. Pulizia cache Python
echo "1. Pulizia cache Python..."
find ~ -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
find ~ -name "*.pyc" -delete 2>/dev/null
find ~ -name "*.pyo" -delete 2>/dev/null
echo "   ✅ Cache Python pulita"
echo ""

# 2. Pulizia cache pip
echo "2. Pulizia cache pip..."
pip cache purge 2>/dev/null || python3 -m pip cache purge 2>/dev/null
echo "   ✅ Cache pip pulita"
echo ""

# 3. Pulizia cache node-gyp (se presente)
echo "3. Pulizia cache node-gyp..."
if [ -d ~/.cache/node-gyp ]; then
    rm -rf ~/.cache/node-gyp/*
    echo "   ✅ Cache node-gyp pulita"
else
    echo "   ⚠️  Cache node-gyp non trovata"
fi
echo ""

# 4. Pulizia log ROS2 vecchi
echo "4. Pulizia log ROS2 vecchi..."
if [ -d ~/.ros/log ]; then
    find ~/.ros/log -type f -mtime +30 -delete 2>/dev/null
    echo "   ✅ Log ROS2 vecchi rimossi"
fi
if [ -d ~/ros2_ws/log ]; then
    find ~/ros2_ws/log -type f -mtime +30 -delete 2>/dev/null
    echo "   ✅ Log ros2_ws vecchi rimossi"
fi
echo ""

# 5. Pulizia build artifacts ROS2 (opzionale - commentato per sicurezza)
# echo "5. Pulizia build ROS2..."
# if [ -d ~/ros2_ws/build ]; then
#     read -p "   Rimuovere build ROS2? (y/N): " -n 1 -r
#     echo
#     if [[ $REPLY =~ ^[Yy]$ ]]; then
#         rm -rf ~/ros2_ws/build/*
#         echo "   ✅ Build ROS2 pulita"
#     fi
# fi
# echo ""

# 6. Pulizia file temporanei
echo "5. Pulizia file temporanei..."
find ~ -type f -name "*.tmp" -mtime +7 -delete 2>/dev/null
find ~ -type f -name "*.log" -mtime +30 -size +10M -delete 2>/dev/null
find ~ -type f -name "*.bak" -mtime +30 -delete 2>/dev/null
echo "   ✅ File temporanei puliti"
echo ""

# 7. Pulizia cache thumbnails
echo "6. Pulizia cache thumbnails..."
if [ -d ~/.cache/thumbnails ]; then
    rm -rf ~/.cache/thumbnails/*
    echo "   ✅ Cache thumbnails pulita"
fi
echo ""

# 8. Verifica node_modules duplicati (solo report)
echo "7. Verifica node_modules..."
node_modules_count=$(find ~ -type d -name node_modules 2>/dev/null | wc -l)
echo "   📦 Trovati $node_modules_count directory node_modules"
echo "   💡 Per rimuovere node_modules non utilizzati, eseguire manualmente:"
echo "      find ~/pandai_ark -type d -name node_modules -exec du -sh {} \; | sort -hr"
echo ""

# Verifica spazio finale
echo "📊 Spazio finale:"
df -h / | grep -v Filesystem
echo ""

# Calcola spazio liberato
echo "✅ Pulizia completata!"
echo ""
echo "💡 SUGGERIMENTI:"
echo "   - Per liberare più spazio, considera di:"
echo "     1. Rimuovere progetti non utilizzati da ~/projects"
echo "     2. Pulire build artifacts in ~/pandai_ark se non servono"
echo "     3. Rimuovere modelli ML vecchi da ~/pandai_ark/ros/data/models"
echo "     4. Pulire pacchetti snap non utilizzati: snap list --all"
echo ""







