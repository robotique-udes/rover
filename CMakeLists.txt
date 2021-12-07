cmake_minimum_required(VERSION 2.8.3)
project(ros_talon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
  std_msgs
  message_generation
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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    talon_lib
  DEPENDS
    can_msgs
    std_msgs
    roscpp
    message_runtime
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
target_link_libraries(talon_lib
  ${catkin_LIBRARIES}
)

add_dependencies(talon_lib
  ${catkin_EXPORTED_TARGETS}
)
add_executable(direct_drive1 src/direct_drive1.cpp)
add_executable(direct_drive2 src/direct_drive2.cpp)
add_executable(direct_drive3 src/direct_drive3.cpp)
add_executable(direct_drive4 src/direct_drive4.cpp)
add_executable(direct_drive5 src/direct_drive5.cpp)
add_executable(direct_drive6 src/direct_drive6.cpp)
add_executable(servo_pos src/servo_pos.cpp)
target_link_libraries(direct_drive1
  talon_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(direct_drive2
  talon_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(direct_drive3
  talon_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(direct_drive4
  talon_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(direct_drive5
  talon_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(direct_drive6
  talon_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(servo_pos
  talon_lib
  ${catkin_LIBRARIES}
)
