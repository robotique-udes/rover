# ROS-Talon

[ROS](http://www.ros.org/) package to interface with the [Talon SRX](http://www.ctr-electronics.com/talon-srx.html) motor controller from Cross The Road Electronics through CAN protocol using the [socketcan_bridge](http://wiki.ros.org/socketcan_bridge) ROS package.

# Usage and details

## System configuration and hardware setup

In order the run this project, the follow these instructions:

* [Install](http://www.ros.org/install/) a ROS distribution (It has been tested under Kinetic and Melodic).
* [Install](http://wiki.ros.org/ros_canopen?distro=melodic) the ros_canopen package. To do so, run this on the commandline:
...sudo apt install ros-DISTRO-ros_canopen
* [Follow](http://wiki.ros.org/socketcan_interface) these instructions on how to properly setup the USB-CAN interface.

Now you should be all set. [Build](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) this package using catkin, start a ros_core, start the socketcan_bridge node. Connect the USB-CAN interface to Talon's CAN port, make sure that the Talon has a motor attached to it and power up the system. If everything went fine, you should see the Talon status LED's blink Orange, indicating that the CAN bus has been detected, but no command has been issued.

## Topics and services

The package consists of two nodes: direct_drive and servo_pos. When direct_drive node is running, it listens to the ros_talon/motor_percent topic, corresponding to a single integer (std_msgs/Int32) in the range (-100, 100) indicating the output voltage applied to the motor in percent. When servo_pos node is running, it listens to the ros_talon/steering_angle topic, corresponding to a single float (std_msgs/Float32) in the range (-450, 450) degrees. The talon will drive the motor so it keeps the given position. In order to work, this node needs the quadrature encoder attached.

Both nodes will publish the ros_talon/current_position (std_msgs/Float32) and ros_talon/status (ros_talon/Status) topics. Both nodes will also provide two services: find_center and set_PID. find_center receives no input and provides no output. It finds the center using the signals from a mechanical encoder located in the motor output ring. The same mechanism is used to prevent the motor to go beyond +-450 degrees. The set_PID service receives three float32 inputs (Kp, Ki and Kd). These are the values used for the internal PID loop to keep the given position in Servo mode.

## ArbID construction

Analyzing the construction of the ArbID's used on CTRE's Phoenix library to be able to understand/port code from Phoenix, and also write some on my own, I've found the following:

* Bits  0-5  correspond  to  the  device  number,  in  the  range  from  (int)0  to
(int)62.  The device number (int)63 is reserved for general addressing on the ENABLE Frame.

* Bits 6-16 correspond to a code that will identify what kind of data is being
sent/received (STATUS_XX or similar on talon.h).

* Bits 17-29 correspond to the device ID. This is equal to 2 for the TalonSRX device.

* Bits 30-32 are wasted, as ArbID’s are only 29 bits long, but the nearest
data type is 32 bits long (UInt32).

* Don’t really know why, but the bit 19 is included by both, the device type
id and the control/status id.  Also present on the ENABLE Frame.

# Known issues

socketcan_bridge doesn't handle error, so communication completely breaks when an error occurs. This has been observed only when first running the node. socketcan_bridge will crash at least three times before becoming stable. Once it becomes stable no crash has been noted. This is an open issue on ros_canopen develpment. For more information see:

* https://github.com/ros-industrial/ros_canopen/issues/244#issue-248034107
* https://github.com/ros-industrial/ros_canopen/pull/249#issue-140646021
* https://github.com/ros-industrial/ros_canopen/issues/285#issue-328536939

A simple workaround -far from ideal though- is to just start/kill the socketcan_bridge node until it becomes stable. To do so, run can_reset.sh, located under utils folder on this package.
