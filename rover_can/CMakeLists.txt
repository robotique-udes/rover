cmake_minimum_required(VERSION 3.8)
project(rover_can)

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
  src
  ${rover_msgs_INCLUDE_DIRS}
)

file(GLOB_RECURSE ROVER_CAN_HEADERS
    src/rover_can2/*.hpp 
    src/rover_can2/msgs/*.hpp 
    src/rover_can2/drivers/*.hpp
    src/rover_lib2/helpers/*.hpp
    src/rover_lib2/*.hpp)

add_executable(can_master src/can_master.cpp)
add_executable(can_master_new src/can_master_new.cpp
                          ${ROVER_CAN_HEADERS})

ament_target_dependencies(can_master
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

ament_target_dependencies(can_master_new
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          can_master
          can_master_new
        DESTINATION lib/${PROJECT_NAME})

ament_package()
