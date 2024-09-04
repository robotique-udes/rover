# How to use the rover GUI

The GUI is a PyQt5 application connected with ROS2 via a single node. This `ui_node` contains every subscription and publisher.

## Launching the app

To launch the app, run the following command (don't forget to build):

```bash
ros2 run rover_gui main_gui
```

If you encounter this error when building:


```bash
stderr: rover_gui
/usr/lib/python3/dist-packages/setuptools/command/easy_install.py:158: 
EasyInstallDeprecationWarning: easy_install command is deprecated. 
Use build and pip and other standards-based tools.
warnings.warn(
```

You can fix it by running:
```bash
pip install setuptools==58.2.0
```

The error should be resolved upon rebuilding:

```bash
colcon build
```

## Software developpement on the roverGUI

### How to add a widget

TODO

### ROS communication

TODO
