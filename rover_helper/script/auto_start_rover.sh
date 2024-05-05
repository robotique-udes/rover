#!/bin/bash

echo "Starting all rover launchfiles"
echo ${HOME}
source /opt/ros/humble/setup.bash
source /home/${HOME}/ros2_ws/install/local_setup.bash

ros2 launch rover_helper rover.launch.py &
pid1=$!

wait -n

kill -TERM -$pid1
exit $?
