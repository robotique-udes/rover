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

add_subdirectory(../rover_shared_libs/ .build)

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(ddb_control src/DDB/ddb_control_node.cpp)
ament_target_dependencies(ddb_control
                          rclcpp
                          rover_msgs
                          rcl_interfaces)
target_link_libraries(ddb_control rover_shared_libs)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          ddb_control
        DESTINATION lib/${PROJECT_NAME})
  
ament_package()
