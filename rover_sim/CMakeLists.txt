cmake_minimum_required(VERSION 3.8)
project(rover_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${rover_msgs_INCLUDE_DIRS}
)

add_executable(simulation src/simulation_status.cpp)

ament_target_dependencies(simulation
                          rclcpp
                          std_msgs
                          rover_msgs
                          sensor_msgs
                          tf2_ros)

install(
  DIRECTORY 
    launch
    rover_rviz
    rover_urdf
  DESTINATION 
    share/${PROJECT_NAME}/
)

install(TARGETS
          simulation
        DESTINATION lib/${PROJECT_NAME})

ament_package()
