#!/bin/bash
export ROS_MASTER_URI=http://192.168.1.50:11311/

/usr/bin/expect -f <(cat << EOF
# exp_internal 1

spawn ssh rovus@192.168.1.50
expect "assword:"
send "rovus\n"
send "\n"
expect "rovus@ubuntu:"
puts "Killing roscore instance if it already exist"
send "pkill roscore\n"
sleep 3
expect "rovus@ubuntu:"
send "cd catkin_ws/src/rover_control/scripts/\n"
expect "rovus@ubuntu:"
send "./modprobe-setup.sh\n"
expect "assword"
send "rovus\n"
send "\n"
expect "rovus@ubuntu:"
send "roscore &\n"
sleep 3
send "\n"
expect "rovus@ubuntu:"
send "disown\n"
send "\n"
sleep 3
expect "rovus@ubuntu:"
send "^C"
puts "roscore started"
EOF
)

echo "Starting rover"
rqt
