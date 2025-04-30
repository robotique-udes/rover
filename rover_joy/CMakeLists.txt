cmake_minimum_required(VERSION 3.8)
project(rover_joy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rover_msgs REQUIRED)

add_subdirectory(../rover_shared_libs/ .build)

add_executable(joy_formator src/joy_formator.cpp)
ament_target_dependencies(joy_formator
                          rclcpp
                          joy
                          sensor_msgs
                          rover_msgs)
target_link_libraries(joy_formator rover_shared_libs)

add_executable(joy_demux src/joy_demux.cpp)
ament_target_dependencies(joy_demux
                          rclcpp
                          joy
                          rover_msgs)
target_link_libraries(joy_demux rover_shared_libs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          joy_formator
          joy_demux
        DESTINATION lib/${PROJECT_NAME})

ament_package()
