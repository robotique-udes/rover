cmake_minimum_required(VERSION 3.8)
project(rover_can)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../rover_shared_libs/
)

add_subdirectory(../rover_shared_libs/ .build)

add_executable(can_master src/can_master.cpp)
ament_target_dependencies(can_master
                          rclcpp
                          rover_msgs
                          rcl_interfaces)
target_link_libraries(can_master rover_shared_libs)

add_executable(can_master2 src/can_master2.cpp)
ament_target_dependencies(can_master2
                          rclcpp
                          rover_msgs
                          rcl_interfaces)
target_link_libraries(can_master2 rover_shared_libs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          can_master
          can_master2
        DESTINATION lib/${PROJECT_NAME})

ament_package()
