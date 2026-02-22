cmake_minimum_required(VERSION 3.8)
project(rover_auxiliary)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0079 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../deps/cpr
  ${CMAKE_CURRENT_BINARY_DIR}/cpr_build
)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../deps/rover_lib
  ${CMAKE_CURRENT_BINARY_DIR}/rover_lib_build
)

ament_target_dependencies(rover_shared_libs
  rclcpp
  rover_msgs
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(ddb_control src/DDB/ddb_control_node.cpp)
add_executable(camera_manager
  src/CameraManager/manager_node.cpp
  src/CameraManager/arbitration.cpp
)
add_executable(antenna
  src/Antenna/antenna_node.cpp
  src/Antenna/antenna_driver.cpp
  src/Antenna/antenna_commands/base/get_status.cpp
  src/Antenna/antenna_commands/base/get_interface_stats.cpp
  src/Antenna/antenna_commands/base/login.cpp
  src/Antenna/antenna_commands/base/validate_auth.cpp
)

ament_target_dependencies(ddb_control rclcpp rover_msgs rcl_interfaces)
target_link_libraries(ddb_control rover_shared_libs)
target_include_directories(ddb_control PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

ament_target_dependencies(camera_manager rclcpp rover_msgs rcl_interfaces)
target_link_libraries(camera_manager rover_shared_libs ${OpenCV_LIBS})
target_include_directories(camera_manager PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

ament_target_dependencies(antenna rclcpp rover_msgs rcl_interfaces)
target_link_libraries(antenna rover_shared_libs jsoncpp_lib cpr::cpr)
target_include_directories(antenna PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS ddb_control camera_manager antenna
  DESTINATION lib/${PROJECT_NAME})

ament_package()