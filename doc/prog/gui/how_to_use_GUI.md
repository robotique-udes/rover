# How to use the rover GUI

The GUI is a PyQt5 application connected with ROS2 via a single node. This `ui_node` contains every subscription and publisher needed for the GUI.

## Launching the app

To launch the app only, run the following command (don't forget to build):

```bash
ros2 run rover_gui main_gui
```

To run the GUI and the rest of the rover run:

```bash
ros2 launch rover_helper base.launch.py
```

In another terminal (only necessary if not connected with the rover (standalone))

```bash
ros2 launch rover_helper rover.launch.py
```

If you encounter errors when building make sure to update your [dependencies](../../../README.md#dependencies).

## Software development on the roverGUI

### How to add a widget

TODO

### ROS communication

TODO
