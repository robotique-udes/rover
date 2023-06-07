cmake_minimum_required(VERSION 2.8.3)
project(rover_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES rover_control
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_heartbeat src/base_heartbeat.cpp)
target_link_libraries(base_heartbeat ${catkin_LIBRARIES})
add_dependencies(base_heartbeat rover_control_generate_messages_cpp)
