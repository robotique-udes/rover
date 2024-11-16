#!/bin/bash

interface_up() {
    ip link show dev "$1" up &> /dev/null
}

echo "Starting all rover launchfiles"

echo ${HOME}
source /opt/ros/humble/setup.bash
source /home/${HOME}/ros2_ws/install/local_setup.bash

sleep 5

export ROS_DOMAIN_ID=69
ros2 launch rover_helper rover.launch.py &
pid1=$!

wait -n

kill -TERM -$pid1
exit $?
