cmake_minimum_required(VERSION 3.8)
project(rover_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui WebEngineWidgets WebChannel)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-plugins-base-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

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
  ${Python3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/rover_shared_libs/
)

link_directories(${GSTREAMER_LIBRARY_DIRS})

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/ui_generated
    COMMAND bash ${CMAKE_SOURCE_DIR}/ui/generate_ui.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating UI files..."
)

add_custom_target(run_generate_ui ALL DEPENDS ${CMAKE_BINARY_DIR}/ui_generated)

add_executable(main_gui src/gui.cpp

                   src/MainWindow.hpp
                   src/MainWindow.cpp

                   src/SecondaryWindow.hpp
                   src/SecondaryWindow.cpp
                   
                   src/QLogManager.hpp
                   src/QLogManager.cpp
                  
                   src/Global/Constant/StyleSheet.hpp
                   src/Global/Helpers/QHelpers.hpp
                   src/Global/Helpers/QHelpers.cpp

                   src/Global/Helpers/QToastNotification/QToastNotification.cpp
                   src/Global/Helpers/QToastNotification/QToastNotification.hpp
                   src/Global/Helpers/QToastNotification/QNotificationHistoryData.cpp
                   src/Global/Helpers/QToastNotification/QNotificationHistoryData.hpp
                   src/Global/Helpers/QToastNotification/QNotificationShowHistory.cpp
                   src/Global/Helpers/QToastNotification/QNotificationShowHistory.hpp
                   
                   src/Global/Workers/QWorker.hpp
                   src/Global/Workers/QWorker.cpp
                   src/Global/QTmpFolderManager.hpp
                   src/Global/QTmpFolderManager.cpp

                   src/QSshFileExplorer/Worker/LibSshSupportModule.hpp
                   src/QSshFileExplorer/Worker/LibSshSupportModule.cpp
                   src/QSshFileExplorer/Worker/QSshWorker.hpp
                   src/QSshFileExplorer/Worker/QSshWorker.cpp
                   src/QSshFileExplorer/QDownloadedFileManager.hpp
                   src/QSshFileExplorer/QDownloadedFileManager.cpp
                   src/QSshFileExplorer/QFileItem.hpp
                   src/QSshFileExplorer/QFileItem.cpp
                   src/QSshFileExplorer/QFileTransferWidget.hpp
                   src/QSshFileExplorer/QFileTransferWidget.cpp
                   src/QSshFileExplorer/QIconManager.hpp
                   src/QSshFileExplorer/QIconManager.cpp
                   src/QSshFileExplorer/QSshFileExplorerWidget.hpp
                   src/QSshFileExplorer/QSshFileExplorerWidget.cpp
                   src/QSshFileExplorer/QTreeViewExplorer.hpp
                   src/QSshFileExplorer/QTreeViewExplorer.cpp
                        
                   src/QSideBar/QSideBar.hpp
                   src/QSideBar/QSideBar.cpp
                        
                   src/QDashboard/QDashboard.hpp
                   src/QDashboard/QDashboard.cpp
                        
                   src/QNavigation/QNavigation.hpp
                   src/QNavigation/QNavigation.cpp
                   src/Global/Constant/resources.qrc
                        
                   src/QDashboard/QExample.hpp
                   src/QDashboard/QExample.cpp

                   src/QUtilityBarBottom/QUtilityBarBottom.hpp
                   src/QUtilityBarBottom/QUtilityBarBottom.cpp

                   src/QDeviceStatus/QDeviceStatus.hpp
                   src/QDeviceStatus/QDeviceStatus.cpp
                   
                   src/QVideoPlayer/Worker/QGStreamerWorker.cpp
                   src/QVideoPlayer/Worker/QGStreamerWorker.hpp
                   src/QVideoPlayer/QVideoManagerWidget.cpp
                   src/QVideoPlayer/QVideoManagerWidget.hpp
                   src/QVideoPlayer/QVideoPlayerWidget.cpp
                   src/QVideoPlayer/QVideoPlayerWidget.hpp
                   src/QVideoPlayer/Worker/QPlayerWorker.hpp
                   src/QVideoPlayer/Worker/QPlayerWorker.cpp
                   
                )
                  
add_subdirectory(../rover_shared_libs/ .build)

ament_target_dependencies(main_gui
  rclcpp
  rover_msgs
  rcl_interfaces
)

target_link_libraries(main_gui 
  Qt6::Core
  Qt6::Widgets
  Qt6::Gui
  Qt6::WebEngineWidgets
  Qt6::WebChannel
  ${LIBSSH_LIBRARY}
  ${GSTREAMER_LIBRARIES}
  rover_shared_libs
)         

target_sources(main_gui PRIVATE ${RESOURCES})
                      

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS main_gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()