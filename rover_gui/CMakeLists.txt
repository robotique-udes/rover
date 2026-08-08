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
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui WebEngineWidgets WebChannel Svg DBus Charts)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-plugins-base-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
find_package(jsoncpp REQUIRED)

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

                   src/MainWindow.cpp
                   src/SecondaryWindow.cpp
                   src/QLogManager.cpp
                   src/QScreenInhibitor.cpp
                  
                   src/Global/Helpers/QHelpers.cpp
                   src/Global/Helpers/QToastNotification/QToastNotification.cpp
                   src/Global/Helpers/QToastNotification/QNotificationHistoryData.cpp
                   src/Global/Helpers/QToastNotification/QNotificationShowHistory.cpp
                   src/Global/Helpers/QSessionFolderManager/QSessionFolderManager.cpp
                   src/Global/Workers/QWorker.cpp
                   src/Global/QFlowLayout.cpp
                   src/Global/QTmpFolderManager.cpp
                   
                   src/QArbitration/QArbitration.cpp
                   src/QArbitration/QArbitration.cpp

                   src/QDeviceStatus/QDeviceStatus.cpp
                   src/QDeviceStatus/Worker/QStatusWorker.cpp

                   src/QLightsController/QLightsController.cpp

                   src/QNavigation/QNavigation.cpp
                   src/QNavigation/QWaypoint/QWaypointManager.cpp
                   src/QNavigation/Worker/QPathManager.cpp

                   src/QSideBar/QSideBar.cpp

                   src/QSshFileExplorer/Worker/LibSshSupportModule.cpp
                   src/QSshFileExplorer/Worker/QSshWorker.cpp
                   src/QSshFileExplorer/QDownloadedFileManager.cpp
                   src/QSshFileExplorer/QFileItem.cpp
                   src/QSshFileExplorer/QFileTransferWidget.cpp
                   src/QSshFileExplorer/QIconManager.cpp
                   src/QSshFileExplorer/QSshFileExplorerWidget.cpp
                   src/QSshFileExplorer/QTreeViewExplorer.cpp

                   src/QUtilityBarBottom/QUtilityBarBottom.cpp

                   src/QUtilityBarTop/QUtilityBarTop.cpp
                                           
                   src/QVideoPlayer/Worker/QGStreamerWorker.cpp
                   src/QVideoPlayer/QVideoManagerWidget.cpp
                   src/QVideoPlayer/QVideoPlayerWidget.cpp
                   src/QVideoPlayer/QVideoRecorderWidget.cpp
                   src/QVideoPlayer/Worker/QPlayerWorker.cpp
                   src/QVideoPlayer/Worker/QRecordingWorker.cpp
                   src/QVideoPlayer/Worker/QPanoramaWorker.cpp
                   
                   src/QScience/QScience.cpp

                   src/QBmsData/QBmsData.cpp
                   src/QBmsData/QCellWidget.cpp
                   src/QBmsData/QBattChart.cpp

                   src/Global/Constant/resources.qrc
                   ui/icons.qrc
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
  Qt6::Svg
  Qt6::WebEngineWidgets
  Qt6::WebChannel
  Qt6::DBus
  Qt6::Charts
  ${LIBSSH_LIBRARY}
  ${GSTREAMER_LIBRARIES}
  rover_shared_libs
  jsoncpp_lib
)         

target_sources(main_gui PRIVATE ${RESOURCES})
                      

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS main_gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
