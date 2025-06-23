cmake_minimum_required(VERSION 3.8)
project(rover_autonomous)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


add_subdirectory(../rover_shared_libs/ .build)

include_directories(
  include
)

add_executable(goal_manager_node src/goal_manager_node.cpp)


ament_target_dependencies(goal_manager_node
                          rclcpp
                          rover_msgs
                          sensor_msgs
                          pcl_conversions
                          PCL)
                          
target_link_libraries(goal_manager_node rover_shared_libs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
        goal_manager_node
        DESTINATION lib/${PROJECT_NAME})
  
ament_package()
