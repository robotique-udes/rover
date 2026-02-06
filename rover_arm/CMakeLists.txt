cmake_minimum_required(VERSION 3.8)
project(rover_arm)

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
find_package (Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_subdirectory(../deps/rover_lib/ .build)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop
                          rclcpp
                          std_msgs
                          rover_msgs
                          Eigen3)
target_link_libraries(teleop Eigen3::Eigen rover_lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          teleop
        DESTINATION lib/${PROJECT_NAME})
    
ament_package()
