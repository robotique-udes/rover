cmake_minimum_required(VERSION 3.8)
project(rover_joy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0079 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
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

add_executable(joy_formator src/joy_formator.cpp)
ament_target_dependencies(joy_formator rclcpp joy sensor_msgs rover_msgs)
target_link_libraries(joy_formator rover_shared_libs)
target_include_directories(joy_formator PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

add_executable(joy_demux src/joy_demux.cpp)
ament_target_dependencies(joy_demux rclcpp rover_msgs)
target_link_libraries(joy_demux rover_shared_libs)
target_include_directories(joy_demux PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

add_executable(joy_demux_controller src/joy_demux_controller.cpp)
ament_target_dependencies(joy_demux_controller rclcpp rover_msgs)
target_link_libraries(joy_demux_controller rover_shared_libs)
target_include_directories(joy_demux_controller PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  joy_formator
  joy_demux
  joy_demux_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()