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
## ROS Setup
### Ubuntu installation 
The rover team run ros2 "Humble" which runs on "Ubuntu Desktop 22.04.3 LTS" on dual boot. If you are familiar with os installations, you can download the .iso from the [ubuntu website](https://ubuntu.com/download/desktop). 

If you've never setup a dualboot before you can follow this installation [tutorial](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73).
- At step: "Installing updates and third-party software": 
We suggest selecting "Install thrid-party software for graphics and ..."
- At step "Ubuntu dual boot setup": Click on "something else" and only create the [following partition](https://miro.medium.com/v2/resize:fit:720/format:webp/1*NHz494_x-btfTl4tnm0Muw.png), you don't need a swap partition or another partition for your /home folder. Then for the "Device for bootloader installation" select the device with the "Windows bootloader".

### ROS installation and first steps
To install ROS2 on your ubuntu follow the following tutorials:
- [ROS install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

All the following tutorials are necessary steps to learning ROS
- [ROS CLI tools tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS Client librairies](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

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
