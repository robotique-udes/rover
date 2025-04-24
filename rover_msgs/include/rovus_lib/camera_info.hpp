#ifndef __CAMERA__INFO__HPP__
#define __CAMERA__INFO__HPP__

#include <map>
#include <string>

namespace CameraInfo
{
    const std::map<std::string, std::string> CameraName = {
        {"rtsp://rover:roverrover@192.168.144.30:554/1/h264major", "Main"},
        {"rtsp://rover:roverrover@192.168.144.31:554/1/h264major", "Antenna"},
        {"rtsp://rover:roverrover@192.168.144.32:554/1/h264major", "Odometry"},
        {"rtsp://rover:roverrover@192.168.144.35:554/1/h264major", "Gripper1"},
        {"rtsp://rover:roverrover@192.168.144.36:554/1/h264major", "Gripper2"},
    };

    const std::map<std::string, std::string> CameraIP = {
        {"Main", "rtsp://rover:roverrover@192.168.144.30:554/1/h264major"},
        {"Antenna", "rtsp://rover:roverrover@192.168.144.31:554/1/h264major"},
        {"Odometry", "rtsp://rover:roverrover@192.168.144.32:554/1/h264major"},
        {"Gripper1", "rtsp://rover:roverrover@192.168.144.35:554/1/h264major"},
        {"Gripper2", "rtsp://rover:roverrover@192.168.144.36:554/1/h264major"},
    };
}  // namespace CameraInfo

#endif  //__CAMERA__INFO__HPP__