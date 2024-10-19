#!/bin/bash

# Set the script location variable
SCRIPT_LOCATION="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$SCRIPT_LOCATION/ui"
INCLUDE_DIR="$SCRIPT_LOCATION/include"

echo -e "\e[0;34m=== Generating UI files ... ===\e[0m"
# Create the include directory if it doesn't exist
mkdir -p "$INCLUDE_DIR"
rm -rf include/*

# Find all .ui files starting with an underscore
for ui_file in "$UI_DIR"/*.ui; do
    # Check if the file exists to avoid errors if no files are found
    if [[ -f "$ui_file" ]]; then
        # Extract the filename without path
        filename=$(basename "$ui_file")
        
        # Remove the .ui extension and append "UI"
        output_file="UI_${filename%.ui}.h"
        
        # Compile the .ui file to a header file using uic
        uic "$ui_file" -o "$INCLUDE_DIR/$output_file"
        
        echo -e "\e[0;32m[SUCCESS] Compiled $ui_file to $INCLUDE_DIR/$output_file\e[0m"
    else
        echo -e "\e[0;32m[OK] No .ui files found in $UI_DIR, no work done\e[0m"
    fi
done
