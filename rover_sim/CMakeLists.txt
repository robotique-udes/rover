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
  scripts/arm_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(gps_test src/gps_test_pub.cpp)
add_executable(camera_interface_test src/camera_interface_test.cpp)
add_executable(camera_test_pub src/camera_test_pub.cpp)
add_executable(battery_test src/battery_test_pub.cpp)
add_executable(connection_speed_test src/connection_speed_test_pub.cpp)

ament_target_dependencies(gps_test
                          rclcpp
                          rover_msgs)

ament_target_dependencies(camera_interface_test
                          rclcpp
                          rover_msgs)

ament_target_dependencies(camera_test_pub
                          rclcpp
                          rover_msgs)

ament_target_dependencies(battery_test
                          rclcpp
                          rover_msgs)

ament_target_dependencies(connection_speed_test
                          rclcpp
                          rover_msgs)


target_link_libraries(camera_interface_test 
                        rover_shared_libs)

install(TARGETS
        battery_test
        connection_speed_test
        gps_test
        camera_interface_test
        camera_test_pub
        DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
