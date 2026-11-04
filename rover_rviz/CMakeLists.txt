cmake_minimum_required(VERSION 3.8)
project(rover_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(arm_to_joint_state src/arm_to_joint_state.cpp)

target_link_libraries(arm_to_joint_state rover_shared_libs)

ament_target_dependencies(arm_to_joint_state
                          rclcpp
                          rover_msgs
                          sensor_msgs)

install(DIRECTORY launch meshes urdf rviz DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
        arm_to_joint_state
        DESTINATION lib/${PROJECT_NAME})

ament_package()