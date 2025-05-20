cmake_minimum_required(VERSION 3.8)
project(rover_drive_train)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(rover_msgs REQUIRED)

add_subdirectory(../rover_shared_libs/ .build)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop
                          rclcpp
                          std_msgs
                          rover_msgs)
target_link_libraries(teleop rover_shared_libs)

add_executable(arbitration src/arbitration.cpp)
ament_target_dependencies(arbitration
                          rclcpp
                          std_msgs
                          rover_msgs)
target_link_libraries(arbitration rover_shared_libs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          teleop
          arbitration
        DESTINATION lib/${PROJECT_NAME})

ament_package()
