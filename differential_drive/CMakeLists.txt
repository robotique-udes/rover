cmake_minimum_required(VERSION 2.8.3)
project(differential_drive)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

add_message_files(
  FILES
  VelocityTargets.msg
  Encoders.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)