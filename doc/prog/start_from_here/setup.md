# Table of content

- [Table of content](#table-of-content)
- [Introduction](#introduction)
- [ROS Setup](#ros-setup)
  - [Ubuntu installation](#ubuntu-installation)
  - [ROS installation and first steps](#ros-installation-and-first-steps)
- [bashrc](#bashrc)
      - [Compile from everywhere](#compile-from-everywhere)
      - [Enable ros logging colors](#enable-ros-logging-colors)
- [Cloning the repos](#cloning-the-repos)
- [General advices](#general-advices)

# Introduction

This guide will help you setup your computer for development. Follow each section in order, installing the OS and ROS can take quite some time.

# ROS Setup

## Ubuntu installation

The rover team run ros2 "Humble" which runs on "Ubuntu Desktop 22.04.3 LTS". Running a dual boot is very recommended because virtual machine don't work well with hardware. The next steps will help you setup a ubuntu installation side by side of your windows installation.

If you are familiar with os installations, you can download the .iso from the [ubuntu website](https://ubuntu.com/download/desktop).

If you've never setup a dualboot before, we suggest you follow this installation [tutorial](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73).
  - At step: "Installing updates and third-party software": We suggest selecting "Install third-party software for graphics and ..."
  - At step "Ubuntu dual boot setup": Click on "something else" and only create the [following partition](https://miro.medium.com/v2/resize:fit:720/format:webp/1*NHz494_x-btfTl4tnm0Muw.png), you don't need a swap partition or another partition for your /home folder. Then for the "Device for bootloader installation" select the device with the "Windows bootloader".

## ROS installation and first steps

To install ROS2 on your ubuntu follow the following tutorials:

- [ROS install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

We suggest you to do most of the tutorial below, they take quite a long time to do but are necessary steps to learning ROS features and development tools.

- [ROS CLI tools tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS Client libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

# bashrc

The .basrc file is loaded each time a linux terminal is launched. You'll want to enter multiple command each time you work in ROS, adding them to your .bashrc file will simplify your life. To do so, follow the following steps:

- Open your bashrc file

```bash
sudo nano ~/.bashrc
```

- Navigate to the end with _shift-down_arrow_

- Paste these lines (_ctrl+shift+v_):

```bash
# Additions
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=69 # Set on which DOMAIN_ID ros will talk with other computer, all our computer are set on 69
```
- Save and exit by typing: ctrl+x -> y -> enter (or follow on screen item)
- Finally source the bashrc with it's modification:
```bash
source ~/.bashrc
``` 
---

The next commands are optional but are nice to have in your bashrc:

#### Compile from everywhere
The following cmd *create* a cmd to always compile the right folder no matter where your terminal is. Add this line to the end of your bashrc and you'll be able to compile by entering "b" in any bash terminal, anywhere.

```bash
alias b='pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install && popd > /dev/null'
```

#### Enable ros logging colors
```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

# Cloning the repos

Once you've got you ROS set up correctly and working, that you've done most of the tutorial, you're ready to start exploring the code. To do so you'll need to clone 


# General advices

- When error occurs after changing settings or changing branches, run these cmds in a terminal:
```bash
source ~/.bashrc
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log
source ~/.bashrc
pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install && popd > /dev/null
source ~/.bashrc
pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install && popd > /dev/null
```
