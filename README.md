# Dependencies
* joy
* microROS ([see our micro ros projects distro](https://github.com/robotique-udes/rover_micro/)) 

Run the following commands to download and install depedencies from apt:

```
sudo apt install ros-humble-joy
```

install setuptools version 58.2.0 for compatibility reasons ([further details](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/))

```
pip install setuptools==58.2.0
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

### bashrc
The .basrc file is loaded each time a linux terminal is launched. You should see these lines at the end:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```
To add them follow theses steps:
- Open your bashrc file
```
sudo nano ~/.bashrc
```
- Navigate to the end with *shift-down_arrow*
- Paste these lines (*ctrl+shift+v*):
```
# Additions
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

The following cmd create a cmd to always compile in the ros2_ws no matter where your terminal is. Add this line to the end of your bashrc and you'll be able to compile by entering "b" in any bash terminal anywhere.
You can change the "b" to whatever you like and you'll be able to compile by entering the cmd you set.
```
alias b='pushd . > /dev/null && cd ~/ros2_ws && colcon build && popd > /dev/null'
```


# ESP32 and micro controller ROS developpement:
See corresponding [repository](https://github.com/robotique-udes/rover_micro)