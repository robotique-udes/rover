#ifndef __CAMERA__INFO__HPP__
#define __CAMERA__INFO__HPP__

#include <map>
#include <string>

namespace CameraInfo
{
    const std::map<std::string, std::string> CameraName = {{"rtsp://rover:roverrover@192.168.144.30:554/1/h264major", "Main"},
                                                           {"rtsp://rover:roverrover@192.168.144.31:554/1/h264major", "Antenne"},
                                                        };
}

#endif