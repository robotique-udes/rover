#!/bin/bash

# Set the script location variable
SCRIPT_LOCATION="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$SCRIPT_LOCATION/ui"
INCLUDE_DIR="$SCRIPT_LOCATION/include"

echo -e "\e[0;34m=== Generating UI files ... ===\e[0m"
mkdir -p "$INCLUDE_DIR"
rm -rf include/*

for ui_file in "$UI_DIR"/*.ui; do
    if [[ -f "$ui_file" ]]; then
        filename=$(basename "$ui_file")
        output_file="UI_${filename%.ui}.h"
        /usr/lib/qt6/libexec/uic "$ui_file" -o "$INCLUDE_DIR/$output_file"
        
        echo -e "\e[0;32m[SUCCESS] Compiled $ui_file to $INCLUDE_DIR/$output_file\e[0m"
    else
        echo -e "\e[0;32m[OK] No .ui files found in $UI_DIR, no work done\e[0m"
    fi
done
