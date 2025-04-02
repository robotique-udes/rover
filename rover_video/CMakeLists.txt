cmake_minimum_required(VERSION 3.8)
project(rover_video)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} 
                     ${rover_msgs_INCLUDE_DIRS} )


add_executable( media_server src/camera_node.cpp
                src/video_recording.cpp)

ament_target_dependencies(media_server
                          rclcpp
                          rover_msgs
                          OpenCV)


target_link_libraries(media_server ${OpenCV_LIBS})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS 
            media_server
        DESTINATION lib/${PROJECT_NAME}/)

ament_package()
