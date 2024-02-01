cmake_minimum_required(VERSION 3.8)
project(rover_helper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

include_directories(
  ${rover_msgs_INCLUDE_DIRS}
  include
)

add_executable(debug_printer src/debug_printer.cpp)
add_executable(rover_rosserial_manager src/rover_rosserial_manager.cpp)

ament_target_dependencies(debug_printer
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

ament_target_dependencies(rover_rosserial_manager
                          rclcpp
                          rover_msgs
                          rcl_interfaces)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          debug_printer
          rover_rosserial_manager
        DESTINATION lib/${PROJECT_NAME})

ament_package()
