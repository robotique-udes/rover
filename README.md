# Dependencies
* joy
* socketcan_bridge
* usb_cam_hardware
* usb_cam_controllers

Run the following commands to download and install depedencies from apt:
```
sudo apt install ros-noetic-move-base-msgs ros-noetic-joy ros-noetic-socketcan-bridge ros-noetic-usb-cam-hardware ros-noetic-usb-cam-controllers
```

# Setup
## CAN hotplug
Add the following to `/etc/network/interfaces` file to enable CAN hotplug:
```
allow-hotplug can0
iface can0 can static
        bitrate 1000000
        up ip link set $IFACE txqueuelen 1000
```
If this doesn't work on your machine can you run the following command each time you plug-in the can device:
```
./~/home/catkin_ws/src/rover_control/scripts/modprobe-setup.sh
```
