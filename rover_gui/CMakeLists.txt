cmake_minimum_required(VERSION 3.8)
project(rover_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui)

set(LIBSSH_INCLUDE_DIR "/usr/include/libssh/include")
set(LIBSSH_LIBRARY "/usr/lib/x86_64-linux-gnu/libssh.so")

# Qt requirements for auto compiles of .ui and .qrc files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON) 

include_directories(
  include
  ${rover_msgs_INCLUDE_DIRS}
  ${LIBSSH_INCLUDE_DIR}
)

add_executable(application src/application.cpp )

target_link_libraries(application 
                      Qt6::Widgets
                      Qt6::Gui
                      ${LIBSSH_LIBRARY})

ament_target_dependencies(application
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          application
          DESTINATION lib/${PROJECT_NAME})

ament_package()
