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
find_package(std_msgs REQUIRED)

include_directories(
  ${rover_msgs_INCLUDE_DIRS}
)

add_executable(jog_antenna src/jog_antenna.cpp)
add_executable(autonomus src/autonomus.cpp)
add_executable(arbitration src/arbitration.cpp)

ament_target_dependencies(jog_antenna
                          rclcpp
                          rover_msgs)

ament_target_dependencies(autonomus
                          rclcpp
                          rover_msgs)

ament_target_dependencies(arbitration
                          rclcpp
                          std_msgs
                          rover_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
        jog_antenna
        autonomus
        arbitration
        DESTINATION lib/${PROJECT_NAME})

ament_package()
