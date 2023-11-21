#!/bin/bash

pkill roscore
export ROS_MASTER_URI=http://localhost:11311/
roscore &
rqt
