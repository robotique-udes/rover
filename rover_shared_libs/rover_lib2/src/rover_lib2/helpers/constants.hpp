#ifndef ROVER_LIB2_HELPERS_CONSTANTS_HPP
#define ROVER_LIB2_HELPERS_CONSTANTS_HPP
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#if defined(__linux__)
#include <string>
#include <optional>
#endif  // defined(__linux__)

#if defined(ROS)
#include <rclcpp/qos.hpp>
#include <rover_msgs/msg/joy.hpp>
#endif  // defined(ROS)

#if defined(ROS)
#define QOS_DEFAULT rclcpp::QoS(rclcpp::KeepLast(1))
#define QOS_CAMERA rclcpp::QoS(rclcpp::KeepLast(std::to_underlying(Constants::CameraInfo::eCamNames::eLast)))
#endif  // defined(ROS)

namespace Constants
{
    namespace CameraInfo
    {
#if defined(__linux__)

        enum class eCamNames : size_t
        {
            MAIN,
            ANTENNA,
            FRONT_SIDE,
            ARM_TOP,
            ARM_SIDE,
            eLast
        };

        enum class eInfoType : size_t
        {
            NAME,
            URL,
            eLast
        };

        constexpr std::array<std::array<const char*, 2>, std::to_underlying(eCamNames::eLast)> CAMERA_INFO = {{
            {"Main", "rtsp://192.168.144.30:554/1/h264major"},
            {"Antenna", "rtsp://192.168.144.31:554/1/h264major"},
            {"Front-Side", "rtsp://192.168.144.32:554/1/h264major"},
            {"Arm-Top", "rtsp://192.168.144.35:554/1/h264major"},
            {"Arm-Side", "rtsp://192.168.144.36:554/1/h264major"},
        }};

        std::optional<eCamNames> getIdFromURL(const std::string& url_);

        static constexpr const size_t NUMBER_TOPIC_CAMERA_ARBITRATION = 2;

        static constexpr float SEND_COMMAND_PTZ_FREQUENCY = 5.F;
        static constexpr float SEND_CONFIG_PTZ_FREQUENCY = 0.5F;
        static constexpr float SEND_COMMAND_POWER_FREQUENCY = 0.5F;

        static constexpr float RECEIVE_PTZ_STATUS_FREQUENCY = 0.5F;
        static constexpr float RECEIVE_POWER_STATUS_FREQUENCY = 0.5F;
#endif  // defined(__linux__)
    }   // namespace CameraInfo

    namespace AntennaInfo
    {
        enum class eAntennaType : size_t
        {
            BASE = 0,
            ROVER = 1,
            eLast
        };

        static constexpr std::array<const char*, std::to_underlying(eAntennaType::eLast)> ANTENNA_URLS
            = {"https://192.168.144.55", "https://192.168.144.50"};

        template<eAntennaType antenna_>
        constexpr const char* getURL()
        {
            static_assert(std::to_underlying(antenna_) < ANTENNA_URLS.size(), "Invalid antenna index");
            return ANTENNA_URLS[std::to_underlying(antenna_)];
        }
    }  // namespace AntennaInfo

    namespace DriveTrain
    {
        constexpr float SPEED_FACTOR_CRAWLER = 0.2f;
        constexpr float SPEED_FACTOR_NORMAL = 0.5f;
        constexpr float SPEED_FACTOR_TURBO = 1.0f;
        constexpr float SMALLEST_RADIUS = 0.3f;
    }  // namespace DriveTrain

    enum class eGGAQuality : uint8_t
    {
        NO_FIX = 0U,
        GPS = 1U,
        GNSS = 2U,
        RTK = 3U,
    };

    enum class eHeadingQuality : uint8_t
    {
        NO_HEADING = 0U,
        UNRELIABLE = 1U,
        RELIABLE = 2U,
        BEST = 4U,
    };

#if defined(ROS)
    namespace Keybinds
    {
        enum class eJoyInput
        {
            JOYSTICK_LEFT_FRONT = rover_msgs::msg::Joy::JOYSTICK_LEFT_FRONT,
            JOYSTICK_LEFT_SIDE = rover_msgs::msg::Joy::JOYSTICK_LEFT_SIDE,
            JOYSTICK_LEFT_PUSH = rover_msgs::msg::Joy::JOYSTICK_LEFT_PUSH,
            JOYSTICK_RIGHT_FRONT = rover_msgs::msg::Joy::JOYSTICK_RIGHT_FRONT,
            JOYSTICK_RIGHT_SIDE = rover_msgs::msg::Joy::JOYSTICK_RIGHT_SIDE,
            JOYSTICK_RIGHT_PUSH = rover_msgs::msg::Joy::JOYSTICK_RIGHT_PUSH,
            CROSS_UP = rover_msgs::msg::Joy::CROSS_UP,
            CROSS_DOWN = rover_msgs::msg::Joy::CROSS_DOWN,
            CROSS_LEFT = rover_msgs::msg::Joy::CROSS_LEFT,
            CROSS_RIGHT = rover_msgs::msg::Joy::CROSS_RIGHT,
            L1 = rover_msgs::msg::Joy::L1,
            L2 = rover_msgs::msg::Joy::L2,
            R1 = rover_msgs::msg::Joy::R1,
            R2 = rover_msgs::msg::Joy::R2,
            A = rover_msgs::msg::Joy::A,
            B = rover_msgs::msg::Joy::B,
            X = rover_msgs::msg::Joy::X,
            Y = rover_msgs::msg::Joy::Y,
            EXT0 = rover_msgs::msg::Joy::EXT0,
            EXT1 = rover_msgs::msg::Joy::EXT1,
            EXT2 = rover_msgs::msg::Joy::EXT2,
            eLAST
        };

        static_assert(rover_msgs::msg::Joy::MAX >= std::to_underlying(eJoyInput::eLAST));

        namespace JoyDemuxController
        {
            constexpr eJoyInput TOGGLE_BETWEEN_DEMUX = eJoyInput::EXT0;
        }

        namespace DriveTrain
        {
            constexpr eJoyInput DEADMAN_SWITCH = eJoyInput::L1;
            constexpr eJoyInput LINEAR_INPUT = eJoyInput::JOYSTICK_LEFT_FRONT;
            constexpr eJoyInput ANGULAR_INPUT = eJoyInput::JOYSTICK_LEFT_SIDE;
            constexpr eJoyInput MODE_TANK_ANGULAR_INPUT = eJoyInput::JOYSTICK_RIGHT_SIDE;
            constexpr eJoyInput MODE_NORMAL_ENABLE = eJoyInput::R1;
            constexpr eJoyInput MODE_TURBO_ENABLE = eJoyInput::R2;
        }  // namespace DriveTrain

    }   // namespace Keybinds
#endif  // defined(ROS)

}  // namespace Constants

#endif  // ROVER_LIB2_HELPERS_CONSTANTS_HPP
