# Drive train and controls

- [Drive train and controls](#drive-train-and-controls)
  - [Boot and Control Nodes](#boot-and-control-nodes)
    - [Launch files](#launch-files)
    - [Arbitration](#arbitration)
    - [Drive Train](#drive-train)

## Boot and Control Nodes
### Launch files

In order to properly function, the user has to run the following commands to launch the necessary packages.

1. Launch the rover.launch.py file if your testing without the rover. (With the rover, this step is not necessary because this launch file start automatically on the rover main computer)
   ```Bash
   ros2 launch rover_helper rover.launch.py
   ```
2. Launch the base helper package which launches all base rover nodes
   ```Bash
   ros2 launch rover_helper base.launch.py
   ```

### Arbitration

Once packages launched, arbitration needs to be set trough ROS2 srv. This can be done through the gui. Refer to gui doc for more info.

If the gui is down or you do not have access, call the following arbitrations in rqt service caller.

1. /joy/demux_control

   ```Bash
   #controller_type
   CONTROLLER_MAIN = 0 
   CONTROLLER_SECONDARY = 1

   #destination
   DEST_DRIVE_TRAIN = 0
   DEST_ARM = 1
   DEST_ANTENNA = 2
   DEST_NONE = 3
   ```

2. /rover/drive_train/demux_control_cmd

   ```Bash
   #Drive train arbitration destination
   NONE = 0 
   TELEOP = 1
   AUTONOMOUS = 2
   ```

### Drive Train
Controls
 - Deadman Switch -> L1
 - Linear Input -> LEFT ANALOG (FRONT)
 - Angular Input -> LEFT ANALOG (SIDE)
 - Mode Tank -> RIGHT ANALOG (SIDE) *Has priority over regular turning mode*

 Speed Modes
- Normal speed(25%) -> R1
- Turbo Speed(100%) -> R1 + R2
*Crawler speed(1%) is the default speed
