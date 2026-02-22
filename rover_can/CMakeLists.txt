cmake_minimum_required(VERSION 3.8)
project(rover_can)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0079 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../deps/rover_lib
  ${CMAKE_CURRENT_BINARY_DIR}/rover_lib_build
)

ament_target_dependencies(rover_shared_libs
  rclcpp
  rover_msgs
)

include_directories(include)

set(SOURCES_CAN_MASTER
  src/can_master/can_master_node.cpp
  src/can_master/master_device.cpp
  src/can_master/devices/arm_joint.cpp
  src/can_master/devices/camera.cpp
  src/can_master/devices/gnss.cpp
  src/can_master/devices/light.cpp
  src/can_master/devices/propulsion_motors.cpp
  src/can_master/devices/sensor_box.cpp
)

add_executable(can_master ${SOURCES_CAN_MASTER})

ament_target_dependencies(can_master rclcpp rover_msgs rcl_interfaces)

target_include_directories(can_master PRIVATE
  src
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(can_master rover_shared_libs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS can_master
  DESTINATION lib/${PROJECT_NAME})

ament_package()