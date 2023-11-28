cmake_minimum_required(VERSION 2.8.3)
project(rover_antenna)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy
  rover_control_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES rover_control
  CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    joy
    rover_control_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(base_heartbeat src/base_heartbeat.cpp)
#add_dependencies(base_heartbeat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(base_heartbeat ${catkin_LIBRARIES})
