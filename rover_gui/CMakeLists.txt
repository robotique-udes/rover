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
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-plugins-base-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

# Add Python and pybind11 requirements for IPCameraAPI
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

set(LIBSSH_INCLUDE_DIR "/usr/include/libssh/include")
set(LIBSSH_LIBRARY "/usr/lib/x86_64-linux-gnu/libssh.so")

# Qt requirements for auto compiles of .ui and .qrc files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON) 

include_directories(
  src
  ui/include
  ${rover_msgs_INCLUDE_DIRS}
  ${LIBSSH_INCLUDE_DIR}
  ${GSTREAMER_INCLUDE_DIRS}
  ${pybind11_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

link_directories(${GSTREAMER_LIBRARY_DIRS})
qt_add_resources(RTSP_RESOURCES src/Global/Constant/resources.qrc)

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/ui_generated
    COMMAND bash ${CMAKE_SOURCE_DIR}/ui/generate_ui.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating UI files..."
)

add_custom_target(run_generate_ui ALL DEPENDS ${CMAKE_BINARY_DIR}/ui_generated)

# Copy Python module to build directory to ensure it's findable at runtime
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/ipcamera_api.py
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/src/Global/Helpers/ipcamera_api.py
            ${CMAKE_BINARY_DIR}/ipcamera_api.py
    DEPENDS ${CMAKE_SOURCE_DIR}/src/Global/Helpers/ipcamera_api.py
    COMMENT "Copying ipcamera_api.py to build directory"
)
add_custom_target(copy_python_module ALL DEPENDS ${CMAKE_BINARY_DIR}/ipcamera_api.py)

# First create the IPCamera library
add_library(ipcamera_api
    src/Global/Helpers/IPCameraAPI.hpp
    src/Global/Helpers/IPCameraAPI.cpp
)

# Add ROS2 dependencies to the camera API library
ament_target_dependencies(ipcamera_api
    rclcpp
)

target_link_libraries(ipcamera_api
    pybind11::embed
    ${Python3_LIBRARIES}
)

# Then create the main executable
add_executable(main_gui src/gui.cpp

                        src/MainWindow.hpp
                        src/MainWindow.cpp

                        src/SecondaryWindow.hpp
                        src/SecondaryWindow.cpp

                        src/QLogManager.hpp
                        src/QLogManager.cpp

                        src/CameraSettings.hpp
                        src/CameraSettings.cpp
                        
                        src/Global/Constant/StyleSheet.hpp
                        src/Global/Helpers/QHelpers.hpp
                        src/Global/Helpers/QHelpers.cpp
                        src/Global/Workers/QWorker.hpp
                        src/Global/Workers/QWorker.cpp
                        src/Global/QTmpFolderManager.hpp
                        src/Global/QTmpFolderManager.cpp
                        
                        src/Global/Constant/StyleSheet.hpp

                        src/QSshFileExplorer/QIconManager.hpp
                        src/QSshFileExplorer/QIconManager.cpp
                        src/QSshFileExplorer/QFileItem.hpp
                        src/QSshFileExplorer/QFileItem.cpp
                        src/QSshFileExplorer/QFileTransferWidget.hpp
                        src/QSshFileExplorer/QFileTransferWidget.cpp
                        src/QSshFileExplorer/QSshFileExplorerWidget.hpp
                        src/QSshFileExplorer/QSshFileExplorerWidget.cpp
                        src/QSshFileExplorer/QTreeViewExplorer.hpp
                        src/QSshFileExplorer/QTreeViewExplorer.cpp
                        src/QSshFileExplorer/Worker/LibSshSupportModule.hpp
                        src/QSshFileExplorer/Worker/LibSshSupportModule.cpp
                        src/QSshFileExplorer/Worker/QSshWorker.hpp
                        src/QSshFileExplorer/Worker/QSshWorker.cpp
                        src/QSshFileExplorer/QDownloadedFileManager.hpp
                        src/QSshFileExplorer/QDownloadedFileManager.cpp

                        src/QSideBar/QSideBar.hpp
                        src/QSideBar/QSideBar.cpp
                              
                        src/QDashboard/QDashboard.hpp
                        src/QDashboard/QDashboard.cpp
                              
                        src/QNavigation/QNavigation.hpp
                        src/QNavigation/QNavigation.cpp
                              
                        src/QDashboard/QExample.hpp
                        src/QDashboard/QExample.cpp

                        src/QRtspPlayer/QRtspPlayerWidgetHeader.hpp
                        src/QRtspPlayer/QRtspPlayerWidgetCore.cpp
                        src/QRtspPlayer/QRtspPlayerWidgetLogging.cpp
                        src/QRtspPlayer/QRtspPlayerWidgetStreaming.cpp
                        src/QRtspPlayer/QRtspPlayerWidgetUI.cpp
                        src/QRtspPlayer/QGStreamerWorker.cpp
                        src/QRtspPlayer/QGStreamerWorker.hpp
                        src/QRtspPlayer/QLoggingMacros.hpp
                        src/QRtspPlayer/QRtspPlayerWidgetServices.cpp

                        ${RTSP_RESOURCES}
                        )
add_dependencies(main_gui run_generate_ui copy_python_module)

target_link_libraries(main_gui 
                      Qt6::Widgets
                      Qt6::Gui
                      ${LIBSSH_LIBRARY}
                      ${GSTREAMER_LIBRARIES}
                      ipcamera_api)  # Link with our IPCamera library

ament_target_dependencies(main_gui
                          rclcpp
                          rover_msgs
                          rcl_interfaces)

# Install Python module alongside the executable
install(FILES src/Global/Helpers/ipcamera_api.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY include DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS main_gui ipcamera_api  # Install both the executable and the library
          DESTINATION lib/${PROJECT_NAME})

ament_package()