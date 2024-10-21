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
set(CMAKE_AUTORCC ON) 
  
include_directories(
  ui/include
  ${rover_msgs_INCLUDE_DIRS}
  ${LIBSSH_INCLUDE_DIR}
)

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/ui_generated
    COMMAND bash ${CMAKE_SOURCE_DIR}/ui/generate_ui.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating UI files..."
)

add_custom_target(run_generate_ui ALL DEPENDS ${CMAKE_BINARY_DIR}/ui_generated)

add_executable(main_gui src/main_gui.cpp
                        
                        src/Global/QWorker.hpp
                        src/Global/QWorker.cpp
                        src/Global/Constant/StyleSheet.hpp

                        src/QSshFileExplorer/QIconManager.hpp
                        src/QSshFileExplorer/QIconManager.cpp
                        src/QSshFileExplorer/QFileItem.hpp
                        src/QSshFileExplorer/QFileItem.cpp
                        src/QSshFileExplorer/QFileTransferWidget.hpp
                        src/QSshFileExplorer/QFileTransferWidget.cpp
                        src/QSshFileExplorer/QSshFileExplorerWidget.hpp
                        src/QSshFileExplorer/QSshFileExplorerWidget.cpp
                        src/QSshFileExplorer/QSshWorker.hpp
                        src/QSshFileExplorer/QSshWorker.cpp)
add_dependencies(main_gui run_generate_ui)

target_link_libraries(main_gui 
                      Qt6::Widgets
                      Qt6::Gui
                      ${LIBSSH_LIBRARY})

ament_target_dependencies(main_gui
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
main_gui
          DESTINATION lib/${PROJECT_NAME})

ament_package()
