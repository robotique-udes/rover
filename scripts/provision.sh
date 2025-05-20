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
sudo apt -y install qt6-tools-dev
sudo apt -y install qt6-webengine-dev
sudo apt -y install qt6-webengine-dev-tools
sudo apt -y install libqt6svg6-dev
sudo apt -y install libqt6webenginecore6-bin
sudo apt -y install python3-gi
sudo apt -y install libssh-dev
sudo apt -y install sl
sudo apt -y install clang-format
sudo apt -y install libopencv-dev
sudo apt -y install openssh-server

echo -e "\e[0;32m[OK]\e[0m"

echo "=== Installing dep from pip ... ==="
pip install --upgrade setuptools==58.2.0
pip install --upgrade empy==3.3.4
pip install python-can
echo -e "\e[0;32m[OK]\e[0m"

echo -e "=== \e[0;32m[SUCCESS]\e[0m Depedencies updated ==="
