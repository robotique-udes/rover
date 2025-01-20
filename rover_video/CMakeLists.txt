cmake_minimum_required(VERSION 3.8)
project(rover_video)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(<dependency> REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCES

    rover_video src/cam_controller.cpp
    rover_video src/media_server.cpp
)

add_executable( rover_video ${SOURCES})

target_link_libraries( rover_video ${OpenCV_LIBS} )

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



ament_package()
