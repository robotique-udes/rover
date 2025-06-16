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

add_subdirectory(../rover_shared_libs/ .build)

install(PROGRAMS
  scripts/arm_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(gps_test src/gps_test_pub.cpp)
add_executable(rover_sim src/rover_sim.cpp)

ament_target_dependencies(gps_test
                          rclcpp
                          rover_msgs)

ament_target_dependencies(rover_sim
                          rclcpp
                          rover_msgs)

target_link_libraries(rover_sim rover_shared_libs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
          gps_test
          rover_sim
        DESTINATION lib/${PROJECT_NAME})
ament_package()