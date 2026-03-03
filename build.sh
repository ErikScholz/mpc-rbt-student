#!/bin/bash

# Zastaví skript, pokud jakýkoli příkaz selže
set -e

echo "--- Starting compiling ---"

# 1. Načtení ROS 2 prostředí (aby CMake viděl knihovny)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble wasn't found in /opt/ros/humble/"
    exit 1
fi

# 2. Příprava build adresáře
if [ -d "build" ]; then
    echo "Build folder exists, clearing cache..."
    # Volitelně: rm -rf build/* (pokud chceš úplně čistý build)
fi
mkdir -p build
cd build

# 3. Konfigurace projektu pomocí CMake
echo "Starting cmake stuff..."
cmake ..

# 4. Samotná kompilace
# -j$(nproc) využije všechna dostupná jádra procesoru
echo "Compiling..."
make -j$(nproc)

echo "--- Done and done! ---"
