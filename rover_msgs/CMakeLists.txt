cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

set(INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/..")

set(ROVER_BASE_EXEC_PATH "${INSTALL_PATH}/rover_gui/lib/rover_gui/main_gui")
set(ICON_NAME "exec_icon.svg")
set(ROVER_BASE_ICON_PATH_PREFIX "${INSTALL_PATH}/rover_gui/lib/rover_gui")
set(ROVER_BASE_ICON_PATH "${ROVER_BASE_ICON_PATH_PREFIX}/${ICON_NAME}")
set(DESKTOP_FILE_INSTALL_PATH "$ENV{HOME}/.local/share/applications/rover_base.desktop")

set(DESKTOP_FILE
      "[Desktop Entry]\n"
      "Type=Application\n"
      "Name=Rover Base\n"
      "Exec=bash -c 'source ~/.bashrc && ${ROVER_BASE_EXEC_PATH}'\n"
      "Icon=${ROVER_BASE_ICON_PATH}\n"
      "Terminal=true\n"
      "StartupWMClass=Rover Base"
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AntennaCmd.msg"
  "msg/ArmMsg.msg"
  "msg/Aruco.msg"
  "msg/CameraAngle.msg"
  "msg/CameraControl.msg"
  "msg/CanDeviceStatus.msg"  
  "msg/Compass.msg"
  "msg/DDB_control"
  "msg/DevicePower.msg"
  "msg/DrivetrainArbitration.msg"
  "msg/Gps.msg"
  "msg/GpsPosition.msg"
  "msg/Joy.msg"
  "msg/JoyDemuxStatus.msg"
  "msg/LightControl.msg"
  "msg/PropulsionMotor.msg"
  "msg/ScienceControl.msg"

  "srv/AntennaArbitration.srv"
  "srv/ArucoDetection.srv"
  "srv/CameraControl.srv"
  "srv/CompassCalibration.srv"
  "srv/DDBControl.srv"
  "srv/DriveTrainArbitration.srv"
  "srv/Empty.srv"
  "srv/JoyDemuxSetState.srv"
  "srv/LightControl.srv"
  "srv/NewGpsGoal.srv"
  "srv/PanoControl.srv"
  "srv/RtspStream.srv"
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

file(COPY ressource/${ICON_NAME} DESTINATION ${ROVER_BASE_ICON_PATH_PREFIX}/)
file(WRITE ${DESKTOP_FILE_INSTALL_PATH} ${DESKTOP_FILE})

ament_package()
