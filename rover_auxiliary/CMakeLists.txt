cmake_minimum_required(VERSION 3.8)
project(rover_auxiliary)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)

include_directories(
  include
  ${rover_msgs_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_RTSP_SERVER_INCLUDE_DIRS}
)

add_executable(light_control src/light_control.cpp)
add_executable(compass_calibrator src/compass_calibrator.cpp)
add_executable(goal_manager src/goal_manager.cpp)

add_executable(media_repeter src/media_repeter.cpp)
target_link_libraries(media_repeter
                      ${GSTREAMER_LIBRARIES}
                      ${GSTREAMER_RTSP_SERVER_LIBRARIES}
)

ament_target_dependencies(light_control
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

ament_target_dependencies(goal_manager
                          rclcpp
                          rover_msgs
                          rcl_interfaces
                          ament_index_cpp)

ament_target_dependencies(compass_calibrator
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
        light_control
        compass_calibrator
        goal_manager
        media_repeter
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
        scripts/redistribute_stream.py
        scripts/aruco.py
        scripts/screenshot_service.py
        DESTINATION lib/${PROJECT_NAME})
  
ament_package()
