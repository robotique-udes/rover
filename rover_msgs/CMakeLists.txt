cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)
add_library(lib_exec src/lib.cpp)
ament_export_targets(lib_exec HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/rovus_lib
  DESTINATION include
)

install(
  TARGETS lib_exec
  EXPORT lib_exec
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joy.msg"
  "msg/JoyDemuxStatus.msg"
  
  "srv/JoyDemuxSetState.srv"

  # DEPENDENCIES geometry_msgs # Add packages that above messages depend on
)

ament_package()
