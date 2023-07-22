cmake_minimum_required(VERSION 2.8.3)
project(ros_talon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
  std_msgs
  message_generation
  rover_control_msgs
)

add_service_files(
  DIRECTORY
  srv
  FILES
  SetPID.srv
  FindCenter.srv
)

add_message_files(
  DIRECTORY
  msg
  FILES
  Status.msg
  cmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
    can_msgs
    std_msgs
    roscpp
    message_runtime
    rover_control_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(talon_lib
  src/TalonSRX.cpp
  src/TalonRoutines.cpp
  src/TalonModes.cpp
  src/TalonCfg.cpp
)

add_dependencies(talon_lib ${catkin_EXPORTED_TARGETS} ros_talon_generate_messages_cpp)
target_link_libraries(talon_lib
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_executable(direct_drive src/direct_drive.cpp)
add_dependencies(direct_drive ros_talon_generate_messages_cpp)
target_link_libraries(
  direct_drive
  talon_lib
  ${catkin_LIBRARIES}
)
