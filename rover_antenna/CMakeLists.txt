cmake_minimum_required(VERSION 3.8)
project(rover_antenna)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(
  ${rover_msgs_INCLUDE_DIRS}
)

add_executable(antenna_joy src/antenna_joy.cpp)
add_executable(antenna_auto src/antenna_auto.cpp)
add_executable(antenna_abtr src/antenna_abtr.cpp)
add_executable(antenna_udp src/antenna_udp.cpp)

ament_target_dependencies(antenna_joy
                          rclcpp
                          rover_msgs)

ament_target_dependencies(antenna_auto
                          rclcpp
                          rover_msgs)

ament_target_dependencies(antenna_abtr
                          rclcpp
                          rover_msgs)

ament_target_dependencies(antenna_udp
                          rclcpp
                          rover_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
        antenna_joy
        antenna_auto
        antenna_abtr
        antenna_udp
        DESTINATION lib/${PROJECT_NAME})

ament_package()
