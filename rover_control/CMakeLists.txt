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

add_executable(cmd_2_motors src/cmd_2_motors.cpp)
add_dependencies(cmd_2_motors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_2_motors ${catkin_LIBRARIES})

add_executable(gps_node src/gps_node.cpp)
add_dependencies(gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_node ${catkin_LIBRARIES})

add_executable(panorama_server src/panorama_server.cpp)
add_dependencies(panorama_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(panorama_server ${catkin_LIBRARIES})
