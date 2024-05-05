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

## Automatic start of ROS nodes and watcher (systemd)

All the ros executables are now by default running on the rover when the computer has power and every executable will restart automatically when they crash. To configure this run the following commands:

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

    ExecStart=/home/rover/ros2_ws/src/rover/rover_helper/script/auto_start_rover.sh
    Restart=on-failure

    [Install]
    WantedBy=multi-user.target
    ```

    ExecStart=/opt/ros/humble/bin/ros2 launch rover_drive_train drive_train.launch.py
    ExecStart=/opt/ros/humble/bin/ros2 run rover_gui main_gui

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
