cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(
  include
  ${rover_msgs_INCLUDE_DIRS}
)

add_executable(light_control src/light_control.cpp)

ament_target_dependencies(light_control
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          light_control
        DESTINATION lib/${PROJECT_NAME})

ament_package()
