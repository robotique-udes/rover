#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <rover_msgs/msg/joy.hpp>

#if defined(__linux__)
#include <map>
#include <string>
#endif  // defined(__linux__)

#if defined(__linux__) && defined(RCLCPP_DEBUG)
#include <rclcpp/qos.hpp>
#define QOS_DEFAULT rclcpp::QoS(rclcpp::KeepLast(10))
#endif  // defined(__linux__) && defined(RCLCPP_DEBUG)

namespace Constants
{
    namespace CameraInfo
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
    }   // namespace CameraInfo

    namespace DriveTrain
    {
        constexpr float SPEED_FACTOR_CRAWLER = 0.2f;
        constexpr float SPEED_FACTOR_NORMAL = 0.5f;
        constexpr float SPEED_FACTOR_TURBO = 1.0f;
        constexpr float SMALLEST_RADIUS = 0.3f;
    }  // namespace DriveTrain

    namespace KeyBinding
    {
        constexpr float DEADMAN_SWITCH = rover_msgs::msg::Joy::L1;
        constexpr float LINEAR_INPUT = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT;
        constexpr float ANGULAR_INPUT = rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE;
        constexpr float MODE_TANK_ANGULAR_INPUT = rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE;
        constexpr float MODE_NORMAL_ENABLE = rover_msgs::msg::Joy::R1;
        constexpr float MODE_TURBO_ENABLE = rover_msgs::msg::Joy::R2;

    }  // namespace KeyBinding
}  // namespace Constants

#endif  // CONSTANTS_HPP
