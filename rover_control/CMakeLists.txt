cmake_minimum_required(VERSION 2.8.3)
project(rover_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  move_base_msgs
  joy
  socketcan_bridge
  usb_cam_hardware
  usb_cam_controllers
  ros_talon
  rover_control_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES rover_control
  CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    sensor_msgs
    move_base_msgs
    joy
    socketcan_bridge
    usb_cam_hardware
    usb_cam_controllers
    ros_talon
    rover_control_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_heartbeat src/base_heartbeat.cpp)
target_link_libraries(base_heartbeat ${catkin_LIBRARIES})
add_dependencies(base_heartbeat rover_control_generate_messages_cpp)

add_executable(teleop_joystick src/teleop_joystick.cpp)
add_dependencies(teleop_joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_joystick ${catkin_LIBRARIES})
