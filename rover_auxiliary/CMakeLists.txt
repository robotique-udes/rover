cmake_minimum_required(VERSION 3.8)
project(rover_auxiliary)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

add_subdirectory(../deps/cpr cpr_build)
add_subdirectory(../rover_shared_libs/ .build)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(ddb_control src/DDB/ddb_control_node.cpp)

add_executable(bms_info src/BMS/bms_node.cpp)

add_executable(camera_manager src/CameraManager/manager_node.cpp
                      src/CameraManager/arbitration.cpp)
                      
add_executable(antenna src/Antenna/antenna_node.cpp
                       src/Antenna/antenna_driver.cpp
                       src/Antenna/antenna_commands/base/get_status.cpp
                       src/Antenna/antenna_commands/base/get_interface_stats.cpp
                       src/Antenna/antenna_commands/base/login.cpp
                       src/Antenna/antenna_commands/base/validate_auth.cpp)


target_link_libraries(ddb_control rover_shared_libs)
target_link_libraries(camera_manager rover_shared_libs)
target_link_libraries(bms_info rover_shared_libs)
target_link_libraries(antenna 
                      rover_shared_libs
                      jsoncpp_lib
                      cpr::cpr)

ament_target_dependencies(ddb_control
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

ament_target_dependencies(camera_manager
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

ament_target_dependencies(bms_info
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

ament_target_dependencies(antenna
                          rclcpp
                          rover_msgs
                          rcl_interfaces)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
        ddb_control
        DESTINATION lib/${PROJECT_NAME})
        
install(TARGETS
        camera_manager
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS
        antenna
        DESTINATION lib/${PROJECT_NAME})
  
ament_package()
