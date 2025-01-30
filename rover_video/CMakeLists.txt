cmake_minimum_required(VERSION 3.8)
project(rover_video)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rover_msgs REQUIRED)
# find_package(<dependency> REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/include )
include_directories( ${rover_msgs_INCLUDE_DIRS} )


add_executable( media_server src/media_server.cpp )
add_executable( cam_controller src/cam_controller.cpp )

ament_target_dependencies(media_server
                          rclcpp
                          rover_msgs)

ament_target_dependencies(cam_controller
                          rclcpp
                          rover_msgs)


target_link_libraries( media_server ${OpenCV_LIBS} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
            media_server
            cam_controller
        DESTINATION lib/${PROJECT_NAME}/)

ament_package()
