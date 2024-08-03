# Arm control and simulation setup

## Run simulation with rover_helper

### Run the following command to start rviz with rover_helper nodes

   ```Bash
   ros2 launch rover_helper rover.launch.py simulate_arm:=true
   ```
   The simulation is set to false by default. Launching rover.launch.py without any arguments will launch all nodes except simulation.

# Controls
Just like drive_train controls, the arm cannot be controlled if the deadman (L1) is not engaged.

## Joint Controls
The joint controls are split into the default mode which controls individual joints and the gripper mode which individually controls the gripper's position and orientation.

### Default Control Mode

   - Linear Joint LEFT:  L2
   - Linear Joint RIGHT: R2
   - Rotate J0:          Right Joystick Side
   - Rotate J1:          Left Joystick Front
   - Rotate J2:          Right Joystick Front

### Gripper Control Mode
   - Enter Gripper Control Mode: R1
   - Gripper Tilt: Left Joystick Front
   - Gripper Rotation: Right Joystick Side
   - Open Gripper: R2
   - Close Gripper: L2

## Cartesian controls
The cartesian controls allow to directly control the x, y and z position of the effector and its angle. Note that the different modes are selected by a toggle.

### Default Control Mode (position)
This mode controls the x, y and z position of the end effector

   - Increment X position: Left Joystick Front
   - Increment Y position: Right Cross Button 
   - Decrement Y position: Left Cross Button 
   - Increment Z position: Right Joystick Front
   - Open Gripper: R2
   - Close Gripper: L2

### Default Control Mode (position)
This mode controls the x, y and z position of the end effector

   - Increment X position: Left Joystick Front
   - Increment Y position: Right Cross Button 
   - Decrement Y position: Left Cross Button 
   - Increment Z position: Right Joystick Front
   - Open Gripper: R2
   - Close Gripper: L2

### Rotational Control Mode (rotation)
This mode controls the end effector's angle

   - Rotate around the Y axis: Left Joystick Front
   - Rotate around the Z axis: Right Joystick Side
   - Open Gripper: R2
   - Close Gripper: L2

# Kinematics
In this section, you can find the kinematic equations that regulate the robot's arm movements for cartesian controls.

## Robot arm definition
This sections explains the different variables used in the kinematic equations bellow

### Angles

 - q0: This represents the shift of the linear joint of the robot
 - q1: This represents the rotation around the Z axis of the robot's shoulder
 - q2: This represents the rotation around the Y axis of the robot's shoulder
 - q3: This represents the rotation around the Y axis of the robot's elbow
 - q4: This represents the rotation around the Y axis of the robot's wrist

### Joints
For all variables names Jnx, Jny and Jnz, where n represents the joint in quesion. These constants represent the shift in the specified axis from one joint to the next.




## Direct kinematic equations
The following equations were built with the help of MotionGenesis.
### Joint Position 
#### J0 (Base)
- x: 0
- y: q0
- z: 0

#### J1
- x: J0x
- y: J0y + q0
- z: J0z

#### J2
- x: J0x + J1x*cos(q1) - J1y*sin(q1)
- y: J0y + q0 + J1x*sin(q1) + J1y*cos(q1)
- z: J0z + J1z

#### J3
- x: J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) - sin(q1)*(J1y+J2y)
- y: J0y + q0 + J1x*sin(q1) + cos(q1)*(J1y+J2y) + J2x*sin(q1)*cos(0.5*PI-q2) + J2z*sin(q1)*sin(0.5*PI-q2)
- z: J0z + J1z + J2z*cos(0.5*PI-q2) - J2x*sin(0.5*PI-q2)

#### J4
- x: J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) + J3x*cos(q1)*cos(0.5*PI-q2-q3) + J3z*cos(q1)*sin(0.5*PI-q2-q3) - J3y*sin(q1) - sin(q1)*(J1y+J2y)
- y: J0y + q0 + J1x*sin(q1) + cos(q1)*(J1y+J2y) + J2x*sin(q1)*cos(0.5*PI-q2) + J2z*sin(q1)*sin(0.5*PI-q2) + J3x*sin(q1)*cos(0.5*PI-q2-q3) + J3z*sin(q1)*sin(0.5*PI-q2-q3)
- z: J0z + J1z + J2z*cos(0.5*PI-q2) + J3z*cos(0.5*PI-q2-q3) - J2x*sin(0.5*PI-q2) - J3x*sin(0.5*PI-q2-q3)

#### End Effector Position (Po)
- x: J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) + J3x*cos(q1)*cos(0.5*PI-q2-q3) + J3z*cos(q1)*sin(0.5*PI-q2-q3) - sin(q1)*(J1y+J2y) + J4x*sin(q1)*cos(0.5*PI-q2-q3-q4) + J4z*sin(q1)*sin(0.5*PI-q2-q3-q4)
- y: J0y + q0 + J1x*sin(q1) + cos(q1)*(J1y+J2y) + J2x*sin(q1)*cos(0.5*PI-q2) + J2z*sin(q1)*sin(0.5*PI-q2) + J3x*sin(q1)*cos(0.5*PI-q2-q3) + J3z*sin(q1)*sin(0.5*PI-q2-q3) + J4x*sin(q1)*cos(0.5*PI-q2-q3-q4) + J4z*sin(q1)*sin(0.5*PI-q2-q3-q4)
- z: J0z + J1z + J2z*cos(0.5*PI-q2) + J3z*cos(0.5*PI-q2-q3) + J4z*cos(0.5*PI-q2-q3-q4) - J2x*sin(0.5*PI-q2) - J3x*sin(0.5*PI-q2-q3) - J4x*sin(0.5*PI-q2-q3-q4)

### Orientation Angles
- Alpha (angle between Pz and nz): acos(cos(0.5*PI-q2-q3-q4))
- Psi (angle between Py and ny): acos(cos(q1))

