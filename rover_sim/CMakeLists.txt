cmake_minimum_required(VERSION 3.8)
project(rover_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

install(PROGRAMS
  scripts/arm_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(gps_test src/gps_test_pub.cpp)

install(TARGETS
          gps_test
        DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(gps_test
                          rclcpp
                          rover_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
