#ifndef ROVER_AUXILIARY_SIK_RADIO_NODE_HPP
#define ROVER_AUXILIARY_SIK_RADIO_NODE_HPP

#include "rover_lib2/communication/Serial/serial_com.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/sik_status.hpp>
#include <rover_msgs/srv/set_comms_link.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <filesystem>

DEFINE_LOG_NODE(SikRadio, Logger::eNodeState::ON);

class SikRadioNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_SIK_STATUS = "/rover/antenna/sik_status";
    static constexpr const char* TOPIC_JOY_DRIVE = "/base/joy/drive_train";
    static constexpr const char* TOPIC_FALLBACK_JOY = "/rover/fallback/joy";
    static constexpr const char* TOPIC_BASE_HEARTBEAT = "/base/heartbeat";
    static constexpr const char* TOPIC_COMMS_MODE = "/rover/network/comms_mode";
    static constexpr const char* SERVICE_SET_COMMS = "/rover/network/set_comms_link";

#pragma pack(push, 1)
    struct sCompactJoyPacket
    {
        uint8_t magic1{0xAA};
        uint8_t magic2{0x55};
        uint8_t msg_type{0x01};
        int16_t linear{0};
        int16_t angular{0};
        uint8_t deadman{0};
        uint8_t mode_turbo{0};
        uint8_t checksum{0};
    };

    struct sHeartbeatPacket
    {
        uint8_t magic1{0xAA};
        uint8_t magic2{0x55};
        uint8_t msg_type{0x02};
        uint8_t checksum{0x02};
    };

    struct sMavlinkRadioStatusV1
    {
        uint16_t rxerrors;
        uint16_t fixed;
        uint8_t rssi;
        uint8_t remrssi;
        uint8_t txbuf;
        uint8_t noise;
        uint8_t remnoise;
    };
#pragma pack(pop)

  public:
    explicit SikRadioNode();

  private:
    void initNode();
    bool connectSerial();
    void disconnectSerial();
    void loopReceive();
    void sendJoyPacket(const rover_msgs::msg::Joy& msg_);
    void sendHeartbeatPacket();
    void parseBuffer();
    void publishSikStatus(float rssi_, float remRssi_, uint8_t txbuf_, uint8_t noise_, bool connected_);

    void cbSetComms(const std::shared_ptr<rover_msgs::srv::SetCommsLink::Request> req_,
                    std::shared_ptr<rover_msgs::srv::SetCommsLink::Response> res_);

    bool _isBaseStation{false};
    uint8_t _activeMode{rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G};
    std::string _serialPort{"/dev/TOBECHANGED"}; // may need to be changed to smt else depending on the system
    eBaudRate _baudEnum{eBaudRate::B_115200};

    std::unique_ptr<SerialCom> _serial;
    std::vector<uint8_t> _rxBuffer;

    rclcpp::Publisher<rover_msgs::msg::SikStatus>::SharedPtr _pubSikStatus;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pubFallbackJoy;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pubHeartbeat;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _pubCommsMode;

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoy;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subHeartbeat;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _subCommsMode;
    rclcpp::Service<rover_msgs::srv::SetCommsLink>::SharedPtr _srvSetComms;

    rclcpp::TimerBase::SharedPtr _timerRx;
    rclcpp::TimerBase::SharedPtr _timerHeartbeat;
    rclcpp::TimerBase::SharedPtr _timerStatusWatchdog;

    rclcpp::Time _lastPacketReceivedTime;
};

#endif  // ROVER_AUXILIARY_SIK_RADIO_NODE_HPP

