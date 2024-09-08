# Table of content

- [Table of content](#table-of-content)
- [Introduction](#introduction)
- [ROS Setup](#ros-setup)
  - [Ubuntu installation](#ubuntu-installation)
  - [ROS installation and first steps](#ros-installation-and-first-steps)
- [bashrc](#bashrc)
      - [Compile from everywhere](#compile-from-everywhere)
      - [Enable ros logging colors](#enable-ros-logging-colors)
- [Git setup and how to use](#git-setup-and-how-to-use)
- [General advices](#general-advices)

# Introduction

This page will help you setup your computer for development. Follow each section in order, keep in mind that installing the OS and ROS can take quite some time, watching a series on the side is recommended.

# ROS Setup

## Ubuntu installation

The rover team run ros2 "Humble" which runs on "Ubuntu Desktop 22.04.3 LTS". Running a dual boot is very recommended because virtual machine don't work well with physical hardware. The next steps will help you setup a ubuntu installation side by side of your windows installation.

If you are familiar with os installations, you can download the .iso from the [ubuntu website](https://ubuntu.com/download/desktop).

If you've never setup a dualboot before, we suggest you follow this installation [tutorial](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73)
  - At step: "Installing updates and third-party software": We suggest selecting "Install third-party software for graphics and ..."
  - At step "Ubuntu dual boot setup": Click on "something else" and only create the [following partition](https://miro.medium.com/v2/resize:fit:720/format:webp/1*NHz494_x-btfTl4tnm0Muw.png) (allow at least 50gb, 75gb is recommended for our project size), you don't need a swap partition or another partition for your /home folder. Then for the "Device for bootloader installation" select the device with the "Windows bootloader".

## ROS installation and first steps

To install ROS2 on your ubuntu follow the following tutorials:

- [ROS install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

We suggest you to do most of the tutorial below, they take quite a long time to do but are necessary steps to learning ROS features and development tools.

- [ROS CLI tools tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS Client libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

# bashrc

The .basrc file is loaded each time a linux terminal is launched. You'll want to enter multiple command each time you work in ROS, adding them to your .bashrc file will simplify your life:

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
- Save and exit by typing: ctrl+x -> y -> enter (or follow on screen commands)
- Finally source the bashrc with it's modification:
```bash
source ~/.bashrc
``` 
---

The next commands are optional but are nice to have in your bashrc:

#### Compile from everywhere
The following cmd *create* a cmd to always compile and build ROS into the right folder no matter where your terminal is. Add this line to the end of your bashrc and you'll be able to compile by entering "b" in any bash terminal, anywhere.

```bash
alias b='pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install && popd > /dev/null'
```

#### Enable ros logging colors
```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

# Git setup and how to use

Once you've got you ROS set up correctly and that you've done most of the tutorials, you need to setup [git](https://git-scm.com/).

The first thing to setup is your [ssh key*](https://usherbrooke.sharepoint.com/:v:/r/sites/Robotique-UdeS/Documents%20partages/Rover%20-%20G%C3%A9n%C3%A9ral/Tutoriel%20nouveaux/Setup%20ssh%20key.mkv?csf=1&web=1&e=Nlbyax), this is needed because only members of the RobotiqueUdeS github organisation can make changes to the our code.

Now you're ready to clone the repository and actually start coding: [This video*](https://usherbrooke.sharepoint.com/:v:/r/sites/Robotique-UdeS/Documents%20partages/Rover%20-%20G%C3%A9n%C3%A9ral/Tutoriel%20nouveaux/Comment%20utiliser%20git%20ish.mkv?csf=1&web=1&e=d0ssRe) explains how you can make a local copy of the code, make modification safely (without breaking anything) and finally merge it to the "release branch".

  *you need to have access to the RobotiqueUdeS MS-Teams to listen to the video, ps: sorry the sound is quite low.

Lastly, make sure to both clone both the *rover* and the *rover_micro* repos. 
- The *rover* repo need to be cloned inside your /src folder inside of your ros environnement. If you followed the tutorials, the path should be *~/ros2_ws/src*.
- The *rover_micro* repo can be cloned anywhere in your computer other than inside your ros environnement (so anywhere other than *~/ros2_ws/\**) 

# General advices

- When error occurs while compiling/building after changing settings or changing branches, run these cmds in a terminal. This cleans your ROS local build:
```bash
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log
source ~/.bashrc
pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install && popd > /dev/null
source ~/.bashrc
pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install && popd > /dev/null
```
