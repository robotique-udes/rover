#ifndef ROVER_LIB2_HELPERS_CONSTANTS_HPP
#define ROVER_LIB2_HELPERS_CONSTANTS_HPP

#if defined(__linux__)
#include <map>
#include <string>
#include <optional>
#endif  // defined(__linux__)

#if defined(ROS)
#include <rclcpp/qos.hpp>
#include <rover_msgs/msg/joy.hpp>
#endif  // defined(ROS)

#if defined(ROS)
#define QOS_DEFAULT rclcpp::QoS(rclcpp::KeepLast(10))
#endif  // defined(ROS)

namespace Constants
{
    namespace CameraInfo
    {
#if defined(__linux__)
        const std::map<std::string, std::string, std::less<>> CAMERA_URL_MAP = {
            {"Main", "rtsp://192.168.144.30:554/1/h264major"},
            {"Antenna", "rtsp://192.168.144.31:554/1/h264major"},
            {"Front-Side", "rtsp://192.168.144.32:554/1/h264major"},
            {"Arm-Top", "rtsp://192.168.144.35:554/1/h264major"},
            {"Arm-Side", "rtsp://192.168.144.36:554/1/h264major"},
        };

        /**
         * @brief
         * @param url_ URL of the camera
         * @param rName_ Overwrite value if found
         * @return Success on camera name found
         */
        bool getNameFromURL(const std::string& url_, std::string& rName_);
#endif  // defined(__linux__)
    }   // namespace CameraInfo

    namespace AntennaInfo
    {
#if defined(__linux__)
        enum class eAntennaType : size_t
        {
            Base = 0,
            Rover = 1,
            eLast
        };

        static constexpr std::array<const char*, std::to_underlying(eAntennaType::eLast)> ANTENNA_URLS
            = {"https://192.168.144.55", "https://192.168.144.50"};

        constexpr const char* getURL(eAntennaType antenna_)
        {
            return (std::to_underlying(antenna_) < ANTENNA_URLS.size()) ? 
            ANTENNA_URLS.at(std::to_underlying(antenna_)) : nullptr;
        }
#endif  // defined(__linux__)
    }   // namespace AntennaInfo

    namespace DriveTrain
    {
        constexpr float SPEED_FACTOR_CRAWLER = 0.2f;
        constexpr float SPEED_FACTOR_NORMAL = 0.5f;
        constexpr float SPEED_FACTOR_TURBO = 1.0f;
        constexpr float SMALLEST_RADIUS = 0.3f;
    }  // namespace DriveTrain

#if defined(__linux__) && defined(ROS)
    namespace DriveTrain::KeyBinding
    {
        constexpr uint8_t DEADMAN_SWITCH = rover_msgs::msg::Joy::L1;
        constexpr uint8_t LINEAR_INPUT = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT;
        constexpr uint8_t ANGULAR_INPUT = rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE;
        constexpr uint8_t MODE_TANK_ANGULAR_INPUT = rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE;
        constexpr uint8_t MODE_NORMAL_ENABLE = rover_msgs::msg::Joy::R1;
        constexpr uint8_t MODE_TURBO_ENABLE = rover_msgs::msg::Joy::R2;
    }   // namespace DriveTrain::KeyBinding
#endif  // defined(__linux__) && defined(ROS)
}  // namespace Constants

#endif  // ROVER_LIB2_HELPERS_CONSTANTS_HPP
