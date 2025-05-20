#!/bin/bash

interface_up() {
    ip link show dev "$1" up &> /dev/null
}

echo "Starting all rover launchfiles"
echo "Waiting 2s"
sleep 2
echo "Done, starting!"

source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/local_setup.bash
ros2 launch rover_msgs rover.launch.py &
pid1=$!

wait -n

kill -TERM -$pid1
exit $?
