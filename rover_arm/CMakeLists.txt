cmake_minimum_required(VERSION 3.8)
project(rover_arm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0079 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../deps/rover_lib
  ${CMAKE_CURRENT_BINARY_DIR}/rover_lib_build
)

ament_target_dependencies(rover_shared_libs
  rclcpp
  rover_msgs
)

include_directories(include)

add_executable(teleop src/teleop.cpp)

ament_target_dependencies(teleop
  rclcpp
  std_msgs
  rover_msgs
)

target_link_libraries(teleop
  Eigen3::Eigen
  rover_shared_libs  # was rover_lib, must match the target name in rover_lib/CMakeLists.txt
)

target_include_directories(teleop PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()