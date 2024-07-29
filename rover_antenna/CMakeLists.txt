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

add_executable(antennaJoy src/antennaJoy.cpp)
add_executable(antennaAuto src/antennaAuto.cpp)
add_executable(antennaAbtr src/antennaAbtr.cpp)
add_executable(antennaUDP src/antennaUDP.cpp)

ament_target_dependencies(antennaJoy
                          rclcpp
                          rover_msgs)

ament_target_dependencies(antennaAuto
                          rclcpp
                          rover_msgs)

ament_target_dependencies(antennaAbtr
                          rclcpp
                          std_msgs
                          rover_msgs)

ament_target_dependencies(antennaUDP
                          rclcpp
                          rover_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
        antennaJoy
        antennaAuto
        antennaAbtr
        antennaUDP
        DESTINATION lib/${PROJECT_NAME})

ament_package()
