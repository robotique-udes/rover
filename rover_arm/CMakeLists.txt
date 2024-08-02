cmake_minimum_required(VERSION 3.8)
project(rover_arm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)

include_directories(
  ${rover_msgs_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(teleop src/teleop.cpp)

ament_target_dependencies(teleop
                          rclcpp
                          std_msgs
                          rover_msgs
                          Armadillo)

target_link_libraries(teleop
                      ${ARMADILLO_LIBRARIES}
                      ${LAPACK_LIBRARIES}
                      ${BLAS_LIBRARIES})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
          teleop
        DESTINATION lib/${PROJECT_NAME})

ament_package()
