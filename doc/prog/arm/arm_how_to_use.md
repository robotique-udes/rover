# WARNING

This file needs a big overhaul. Outdated information was removed and this is why there's a lot of holes but don't expect all remaining information to be accurate 

## Run simulation with rover_helper

### Run the following command to start rviz with rover_helper nodes

   **As of right now**, arm simulation is in an unstable state and needs to be reworked.

   ```Bash
   ros2 launch rover_helper rover.launch.py simulate_arm:=true
   ```

   The simulation is set to false by default. Launching rover.launch.py without any arguments will launch all nodes except simulation.

# Controls

Just like drive_train controls, the arm cannot be controlled if the deadman (L1) is not engaged.

## Joint Controls

TODO

## Cartesian controls

TODO

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

For all variables names Jnx, Jny and Jnz, where n represents the joint in question. These constants represent the shift in the specified axis from one joint to the next. From the previous joints base.

## Direct kinematic equations

The following equations were built with the help of MotionGenesis.

### Joint Position from center of the arm linear actuator (Bases)

#### J0
```yaml
- x: 0
- y: q0
- z: 0
```

#### J1
```yaml
- x: J0x
- y: J0y + q0
- z: J0z
```

#### J2
```yaml
- x: J0x + J1x*cos(q1) - J1y*sin(q1)
- y: J0y + q0 + J1x*sin(q1) + J1y*cos(q1)
- z: J0z + J1z
```

#### J3
```yaml
- x: J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) - sin(q1)*(J1y+J2y)
- y: J0y + q0 + J1x*sin(q1) + cos(q1)*(J1y+J2y) + J2x*sin(q1)*cos(0.5*PI-q2) + J2z*sin(q1)*sin(0.5*PI-q2)
- z: J0z + J1z + J2z*cos(0.5*PI-q2) - J2x*sin(0.5*PI-q2)
```

#### J4
```yaml
- x: J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) + J3x*cos(q1)*cos(0.5*PI-q2-q3) + J3z*cos(q1)*sin(0.5*PI-q2-q3) - J3y*sin(q1) - sin(q1)*(J1y+J2y)
- y: J0y + q0 + J1x*sin(q1) + cos(q1)*(J1y+J2y) + J2x*sin(q1)*cos(0.5*PI-q2) + J2z*sin(q1)*sin(0.5*PI-q2) + J3x*sin(q1)*cos(0.5*PI-q2-q3) + J3z*sin(q1)*sin(0.5*PI-q2-q3)
- z: J0z + J1z + J2z*cos(0.5*PI-q2) + J3z*cos(0.5*PI-q2-q3) - J2x*sin(0.5*PI-q2) - J3x*sin(0.5*PI-q2-q3)
```

#### End Effector Position (Po)
```yaml
- x: J0x + J1x*cos(q1) + J2x*cos(q1)*cos(0.5*PI-q2) + J2z*cos(q1)*sin(0.5*PI-q2) + J3x*cos(q1)*cos(0.5*PI-q2-q3) + J3z*cos(q1)*sin(0.5*PI-q2-q3) - sin(q1)*(J1y+J2y) + J4x*sin(q1)*cos(0.5*PI-q2-q3-q4) + J4z*sin(q1)*sin(0.5*PI-q2-q3-q4)
- y: J0y + q0 + J1x*sin(q1) + cos(q1)*(J1y+J2y) + J2x*sin(q1)*cos(0.5*PI-q2) + J2z*sin(q1)*sin(0.5*PI-q2) + J3x*sin(q1)*cos(0.5*PI-q2-q3) + J3z*sin(q1)*sin(0.5*PI-q2-q3) + J4x*sin(q1)*cos(0.5*PI-q2-q3-q4) + J4z*sin(q1)*sin(0.5*PI-q2-q3-q4)
- z: J0z + J1z + J2z*cos(0.5*PI-q2) + J3z*cos(0.5*PI-q2-q3) + J4z*cos(0.5*PI-q2-q3-q4) - J2x*sin(0.5*PI-q2) - J3x*sin(0.5*PI-q2-q3) - J4x*sin(0.5*PI-q2-q3-q4)
```

### Orientation Angles
```yaml
- Alpha: acos(cos(0.5*PI-q2-q3-q4)) # (angle between Pz and nz)
- Psi: acos(cos(q1)) # (angle between Py and ny)
```
