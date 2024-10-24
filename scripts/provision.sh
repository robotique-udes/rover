#!/bin/bash
set -e

echo "=== Updating apt ... ==="
sudo apt -y update
echo -e "\e[0;32m[OK]\e[0m"

echo "=== Installing dep from apt ... ==="
sudo apt -y install pip
sudo apt -y install python3-venv
sudo apt -y install python-is-python3
sudo apt -y install ros-humble-desktop
sudo apt -y install ros-humble-ros-base
sudo apt -y install ros-dev-tools
sudo apt -y install ros-humble-joy
sudo apt -y install can-utils
sudo apt -y install qt6-base-dev
sudo apt -y install python3-gi
sudo apt -y install libssh-dev
sudo apt -y install sl
sudo apt -y install libgstrtspserver-1.0-dev
sudo apt -y install clang-format

echo -e "\e[0;32m[OK]\e[0m"

echo "=== Installing dep from pip ... ==="
pip install --upgrade setuptools==58.2.0
pip install --upgrade empy==3.3.4
pip install --upgrade opencv-python
pip install --upgrade opencv-contrib-python
echo -e "\e[0;32m[OK]\e[0m"

echo -e "=== \e[0;32m[SUCCESS]\e[0m Depedencies updated ==="
