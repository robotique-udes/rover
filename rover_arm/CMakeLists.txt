cmake_minimum_required(VERSION 3.8)
project(rover_arm)

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
find_package(joy REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(
  ${rover_msgs_INCLUDE_DIRS}
)

add_executable(teleop src/teleop.cpp)
add_executable(arbitration src/arbitration.cpp)

ament_target_dependencies(teleop
                          rclcpp
                          std_msgs
                          rover_msgs)

ament_target_dependencies(arbitration
                          rclcpp
                          std_msgs
                          rover_msgs)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          teleop
          arbitration
        DESTINATION lib/${PROJECT_NAME})

ament_package()
