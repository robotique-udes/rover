#!/bin/bash

# Enable strict error handling
set -e

# Finding the .clang-format file
echo "=== Finding .clang-format file... ==="
if [ -z "$(find . -name '.clang-format' -print -quit)" ]; then
    echo "[FAILED] .clang-format file not found in the repository!"
    exit 1
else
    echo -e "\e[0;32m[OK]\e[0m .clang-format file found"
fi

# Detecting lint
echo "=== Applying lint... ==="
find . \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -exec clang-format -style=file -i {} \;

echo -e "\e[0;32m[SUCCESS]\e[0m Linting applied successfully!"
