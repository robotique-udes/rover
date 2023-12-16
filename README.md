# Dependencies
* joy
* microROS ([tutorial](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73)) 

Run the following commands to download and install depedencies from apt:
```
sudo apt install ros-humble-joy
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

## microROS Setup
- TODO: installations steps and commons fixes

## ESP32 Setup
Since were running ROS2, all microcontrollers will run [microROS](https://micro.ros.org/) and all developpement will be in platformIO. This [tutorial](https://www.youtube.com/watch?v=Nf7HP9y6Ovo&t=435s) is a great first step at learning microROS.
### General guidelines
 - Use ESP32 for all microcontrollers projects
 - Develop in VSCode with platformIO
 - DON'T use spaces in platformIO project names (this will cause issues with micro_ros_platformio (see [#119](https://github.com/micro-ROS/micro_ros_platformio/issues/119)) 
 - Use serial transport
 - Run your serial ports at 115200 baud
 - Add your project to rover_micro_projects
 - Define all classes/functions/macro/etc that can be reused in the rover_micro_projects/libs/lib_rover/* to limit copy/paste
 - Don't use dynamic allocation and follow embedded coding guidelines as much as you can

### platformio.ini
The ESP32 we currently own are cheap dev boards and they don't work out of the box with platformio. To fix this modify this line in your platformio.ini file:
```
platform = https://github.com/Jason2866/platform-espressif32.git
```
Your platformIO.ini should look like this after the microROS configuration:
```
[env:nodemcu-32s]
platform = https://github.com/Jason2866/platform-espressif32.git
framework = arduino
board = nodemcu-32s
monitor_speed = 115200
board_microros_transport = serial
board_microros_distro = humble

lib_deps = 
    rover_lib=symlink:///home/phil/ros2_ws/src/rover/rover_micro_projects/libs
    https://github.com/micro-ROS/micro_ros_platformio.git
```

### Buying guide
- TODO
