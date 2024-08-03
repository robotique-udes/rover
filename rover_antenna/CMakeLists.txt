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

add_executable(joy_mode src/joy_mode.cpp)
add_executable(auto_mode src/auto_mode.cpp)
add_executable(arbitration src/arbitration.cpp)
add_executable(udp src/udp.cpp)

ament_target_dependencies(joy_mode
                          rclcpp
                          rover_msgs)

ament_target_dependencies(auto_mode
                          rclcpp
                          rover_msgs)

ament_target_dependencies(arbitration
                          rclcpp
                          rover_msgs)

ament_target_dependencies(udp
                          rclcpp
                          rover_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
        joy_mode
        auto_mode
        arbitration
        udp
        DESTINATION lib/${PROJECT_NAME})

ament_package()
