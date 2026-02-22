cmake_minimum_required(VERSION 3.8)
project(rover_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

cmake_policy(SET CMP0079 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../deps/rover_lib
  ${CMAKE_CURRENT_BINARY_DIR}/rover_lib_build
)

ament_target_dependencies(rover_shared_libs
  rclcpp
  rover_msgs
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(media_server
  src/camera_node.cpp
  src/video_recording.cpp
)
ament_target_dependencies(media_server rclcpp rover_msgs)
target_link_libraries(media_server ${OpenCV_LIBS} rover_shared_libs)
target_include_directories(media_server PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

add_executable(aruco_detection
  src/aruco_detection/aruco_detection_node.cpp
  src/aruco_detection/detection.cpp
  src/aruco_detection/process_frame.cpp
  src/aruco_detection/image_capture.cpp
)
ament_target_dependencies(aruco_detection rclcpp rover_msgs rcl_interfaces)
target_link_libraries(aruco_detection ${OpenCV_LIBS} rover_shared_libs)
target_include_directories(aruco_detection PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

add_executable(panorama
  src/panorama_manager.cpp
  src/panorama_processor.cpp
)
ament_target_dependencies(panorama rclcpp rover_msgs rcl_interfaces)
target_link_libraries(panorama ${OpenCV_LIBS} rover_shared_libs)
target_include_directories(panorama PRIVATE
  $<TARGET_PROPERTY:rover_shared_libs,INTERFACE_INCLUDE_DIRECTORIES>
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  media_server
  aruco_detection
  panorama
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()