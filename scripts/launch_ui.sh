#!/bin/bash

set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source $WORKSPACE_DIR/install/local_setup.bash
ros2 launch rover_gui gui.launch.py
