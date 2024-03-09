cmake_minimum_required(VERSION 3.8)
project(rover_can)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(rcl_interfaces REQUIRED)
# find_package(rover_msgs REQUIRED)

include_directories(
  include
  # ${rover_msgs_INCLUDE_DIRS}
)

add_executable(can_translator src/can_translator.cpp)

ament_target_dependencies(can_translator
                          rclcpp
                          # rover_msgs
                          rcl_interfaces)


# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          can_translator
        DESTINATION lib/${PROJECT_NAME})

ament_package()
