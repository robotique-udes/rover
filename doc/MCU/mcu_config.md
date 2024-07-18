# MCU Config

## Table of content

- [MCU Config](#mcu-config)
  - [Table of content](#table-of-content)
  - [PC Specs](#pc-specs)
  - [Config](#config)
  - [Automatic start of ROS nodes and watcher (systemd)](#automatic-start-of-ros-nodes-and-watcher-systemd)
    - [Disable auto start](#disable-auto-start)
    - [Enable auto start](#enable-auto-start)

## PC Specs

## Config

- Ubuntu 22.04
- ros2 humble

## Bashrc
Last updated 2024-07-17

```bash
# Additions
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

alias b='pushd . > /dev/null && cd ~/ros2_ws && colcon build --symlink-install >

export RCUTILS_COLORIZED_OUTPUT=1
export ROS_DOMAIN_ID=69

alias rover_stop='sudo systemctl stop rover_ros_autostart.service'
alias rover_start='sudo systemctl start rover_ros_autostart.service'
```

## Automatic start of ROS nodes and watcher (systemd)

All the ros executables are now by default running on the rover when the computer has power and every executable will restart automatically when they crash. 

Use these commands to stop this service or to restart it (without rebooting)

```bash
rover_stop # Kill the service
rover_start # Start the service
```
### Setup

To configure/install the service run the following commands:

1. Open a text editor

    ```bash
    sudo nano /etc/systemd/system/rover_ros_autostart.service
    ```

2. Paste the following script and change the "**USERNAME**":

    ```ini
    [Unit]
    Description="rover layer"

    [Service]
    Environment="HOME=**USERNAME**"
    Environment="ROS_DOMAIN_ID=69"

    ExecStart=/home/**USERNAME**/ros2_ws/src/rover/rover_helper/script/auto_start_rover.sh
    Restart=on-failure

    [Install]
    WantedBy=multi-user.target

    ExecStart=/opt/ros/humble/bin/ros2 launch rover_drive_train drive_train.launch.py
    ExecStart=/opt/ros/humble/bin/ros2 run rover_gui main_gui
    ```

3. Reload systemd to add the script and enable it

    ```bash
    sudo systemctl daemon-reload
    sudo systemctl enable rover_ros_autostart.service
    sudo systemctl start rover_ros_autostart.service
    ```

### Disable auto start

When debugging it'll probably be nice to disable the service, here's how you can do it:

```bash
sudo systemctl mask rover_ros_autostart.service
sudo systemctl daemon-reload

```

### Enable auto start

```bash
sudo systemctl unmask rover_ros_autostart.service
sudo systemctl daemon-reload
sudo systemctl start rover_ros_autostart.service
```
