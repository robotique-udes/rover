## Packages

* rover_cam: package for camera control and panorama
* rover_control: package for control configuration and teleop
* rover_description: URDF robot description
* rover_viz: Rviz and Mapviz visualization of the robot
* rover_nav: navigation configurations
* rover_gazebo: gazebo simulation of the robot

## Setup
### Joystick/Gamepad setup
Follow this [tutorial](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) and in `rover_control/launch/teleop_joystick.launch` make sure the device name is correct.

## Dependencies

The following packages need to be installed in order for the rover to work properly. Most of them can be installed automatically with by running `rosdep install --from-paths src --ignore-src -r -y` in the root of the catkin workspace.

* realsense2_camera `sudo apt install ros-melodic-realsense2-camera`
* realsense2_description `sudo apt install ros-melodic-realsense2-description`
* robot_localization `sudo apt install ros-melodic-robot-localization`
* [mapviz](https://swri-robotics.github.io/mapviz/)
* hector_gazebo_plugins `sudo apt install ros-melodic-hector-gazebo-plugins`
* rviz_imu_plugin `sudo apt install ros-melodic-rviz-imu-plugin`
* joy `sudo apt install ros-melodic-joy`
* teleop_twist_joy `sudo apt install ros-melodic-teleop-twist-joy`
* teleop_twist_keyboard `sudo apt install ros-melodic-teleop-twist-keyboard`
* [rover_udes](https://github.com/robotique-udes/rover_udes)
* [rover_base](https://github.com/robotique-udes/rover_base)
* [rover_drivers](https://github.com/robotique-udes/rover_drivers)
* rosserial_arduino `sudo apt install ros-melodic-rosserial-arduino`

