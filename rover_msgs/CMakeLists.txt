cmake_minimum_required(VERSION 2.8.3)
project(rover_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
 FILES
 motor_cmd.msg
 velocity_targets.msg
 gps.msg
)

add_service_files(
 FILES
 camera_control.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_generation
)
