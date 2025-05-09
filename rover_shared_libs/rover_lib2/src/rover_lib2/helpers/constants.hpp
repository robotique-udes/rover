#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#if defined(__linux__)
#include <map>
#include <string>
#endif  // defined(__linux__)

namespace Constants::CameraInfo
{
#if defined(__linux__)
    const std::map<std::string, std::string> CAMERA_URL_MAP = {
        {"Main", "rtsp://192.168.144.30:554/1/h264major"},
        {"Antenna", "rtsp://192.168.144.31:554/1/h264major"},
        {"Front-Side", "rtsp://192.168.144.32:554/1/h264major"},
        {"Arm-Top", "rtsp://192.168.144.35:554/1/h264major"},
        {"Arm-Side", "rtsp://192.168.144.36:554/1/h264major"},
    };

    /**
     * @brief Tries to find a name from a camera URL
     *
     * @param url_ URL of the camera
     * @param rName_ Overwrite value if found
     * @return Success on camera name found
     */

    bool getNameFromURL(const std::string& url_, std::string& rName_);
#endif  // defined(__linux__)
}  // namespace Constants::CameraInfo

#endif  // CONSTANTS_HPP
