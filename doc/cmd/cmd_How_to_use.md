# Drive train and controls

- [Drive Train and Controls](#drive-train-and-controls)
    - [Boot Control Nodes](#boot-ctrl-nodes)
        - [Pkg Dependencies](#pgk-dependencies)
        - [Arbitration](#arbitration)
    - [Controls](#controls)
        - [Drive Train](#drive-train)
        - [Science Module](#science-module)

## Boot and Control Nodes
### Pkg Dependencies

In order to properly function, the user has to run the following commands to launch the necessary packages. (make sure you are in your /ros2_ws).

1. Launch the security package which contains base and rover hearbteat
   ```Bash
   ros2 launch rover_security security.launch.py
   ```
2. Launch the base helper package which launches all base rover nodes
   ```Bash
   ros2 launch rover_helper base.launch.py
   ```
3. Launch the drive train package which contains motor control logic and arbitration node
   ```Bash
   ros2 launch rover_drive_train drive_train.launch.py
   ```
4. Launch the science package which contains motor controls for science module
   ```Bash
   ros2 launch rover_science science.launch.py
   ```

### Arbitration

Once packages launched, arbitration needs to be set trough ROS2 srv. This can be done through the gui. Refer to gui doc for more info.

If gui is down or you do not have access, call the following arbitrations in rqt service caller.

1. /joy/demux_control

   ```Bash
   #controller_type
   CONTROLLER_MAIN = 0 
   CONTROLLER_SECONDARY = 1

   #destination
   DEST_DRIVE_TRAIN = 0
   DEST_ARM = 1*
   DEST_ANTENNA = 2
   DEST_NONE = 3
   ```

2. IF USING DRIVE TRAIN /rover/drive_train/demux_control_cmd

   ```Bash
   #Drive train arbitration desitnation
   NONE = 0 
   TELEOP = 1
   AUTONOMOUS = 2
   ```
*The DEST_ARM destination currently controls the science module

## Controls
Refer to the following image for controls

![PS4 controller mappings](https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.juridiconline.com%2F%3Fk%3Dgroup-7-one-handed-playstation-controller-%25E2%2580%2593-tek-bb-5MND8ltq&psig=AOvVaw0m21j40vn6dBbM2G3yluEe&ust=1715376189459000&source=images&cd=vfe&opi=89978449&ved=0CBIQjRxqFwoTCKjMlbXAgYYDFQAAAAAdAAAAABBL)

### Drive Train
Controls
 - Deadman Switch -> L1
 - Linear Input -> LEFT ANALOG (FRONT)
 - Angular Input -> LEFT ANALOG (SIDE)
 - Mode Tank -> RIGHT ANALOG (SIDE) *Has priority over regular turning mode*

 Speed Modes
- Normal speed(25%) -> R1
- Turno Speed(100%) -> R1 + R2
*Crawler speed(1%) is set as default
 
 ### Science Module

Controls
 - Module Up -> DPAD UP
 - Module Down -> DPAD DOWN

Drill
 - Drill In ->  CROSS
 - Drill Out -> TRIANGLE
