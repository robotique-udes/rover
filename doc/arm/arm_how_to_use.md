# Arm control and simulation setup

## Run simulation with rover_helper

### Run the following command to start rviz with rover_helper nodes

   ```Bash
   ros2 launch rover_helper rover.launch.py simulate_arm:=true
   ```
   The simulation is set to false by default. Launching rover.launch.py without any arguments will launch all nodes except simulation.

## Control
Just like drive_train controls, the arm cannot be controlled if the deadman (L1) is not engaged.

### Joint Controls
The joint controls are split into the default mode which controls individual joints and the gripper mode which individually controls the gripper's position and orientation.

#### Default Control Mode

   - Linear Joint LEFT:  L2
   - Linear Joint RIGHT: R2
   - Rotate J0:          Right Joystick Side
   - Rotate J1:          Left Joystick Front
   - Rotate J2:          Right Joystick Front

#### Gripper Control Mode
   - Enter Gripper Control Mode: R1
   - Gripper Tilt: Left Joystick Front
   - Gripper Rotation: Right Joystick Side
   - Open Gripper: R2
   - Close Gripper: L2

### Cartesian controls
This feature has yet to be implemented
