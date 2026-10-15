#ifndef ROVER_AUXILIARY_SIK_RADIO_NODE_HPP
#define ROVER_AUXILIARY_SIK_RADIO_NODE_HPP

#include "rover_lib2/communication/Serial/serial_com.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/joy.hpp>
#include <rover_msgs/msg/sik_status.hpp>
#include <rover_msgs/srv/set_comms_link.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

DEFINE_LOG_NODE(SikRadio, Logger::eNodeState::ON);

class SikRadioNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_SIK_STATUS = "/rover/antenna/sik_status";
    static constexpr const char* TOPIC_JOY_DRIVE = "/base/joy/drive_train";
    static constexpr const char* TOPIC_FALLBACK_JOY = "/rover/fallback/joy";
    static constexpr const char* TOPIC_BASE_HEARTBEAT = "/base/heartbeat";
    static constexpr const char* TOPIC_COMMS_MODE = "/rover/network/comms_mode";
    static constexpr const char* TOPIC_EMERGENCY_SNAPSHOT = "/rover/camera/emergency_snapshot";
    static constexpr const char* TOPIC_EMERGENCY_IMAGE = "/rover/camera/emergency_image";
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

    struct sImageChunkPacket
    {
        uint8_t magic1{0xAA};
        uint8_t magic2{0x55};
        uint8_t msg_type{0x03};
        uint16_t frame_id{0};
        uint8_t chunk_idx{0};
        uint8_t total_chunks{0};
        uint8_t chunk_len{0};
        uint8_t data[180]{0};
        uint8_t checksum{0};
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
    std::string resolveSerialPort();
    bool connectSerial(const std::string& port = "");
    void disconnectSerial();
    void loopReceive();
    void sendJoyPacket(const rover_msgs::msg::Joy& msg_);
    void sendHeartbeatPacket();
    void parseBuffer();
    void publishSikStatus(float rssi_, float remRssi_, uint8_t txbuf_, uint8_t noise_, bool connected_);

    // Emergency camera chunk handling
    static uint8_t computeImageChunkChecksum(const sImageChunkPacket& pkt);
    void queueEmergencySnapshot(const sensor_msgs::msg::CompressedImage& msg_);
    void loopSendImageChunk();
    void handleImageChunk(const sImageChunkPacket& chunk);

    void cbSetComms(const std::shared_ptr<rover_msgs::srv::SetCommsLink::Request> req_,
                    std::shared_ptr<rover_msgs::srv::SetCommsLink::Response> res_);

    bool _isBaseStation{false};
    uint8_t _activeMode{rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G};
    std::string _serialPort{"/dev/ttyUSB0"};
    std::string _activeSerialPort{""};
    eBaudRate _baudEnum{eBaudRate::B_115200};

    std::unique_ptr<SerialCom> _serial;
    std::vector<uint8_t> _rxBuffer;

    rclcpp::Publisher<rover_msgs::msg::SikStatus>::SharedPtr _pubSikStatus;
    rclcpp::Publisher<rover_msgs::msg::Joy>::SharedPtr _pubFallbackJoy;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pubHeartbeat;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _pubCommsMode;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _pubEmergencyImage;

    rclcpp::Subscription<rover_msgs::msg::Joy>::SharedPtr _subJoy;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subHeartbeat;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _subCommsMode;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _subEmergencySnapshot;
    rclcpp::Service<rover_msgs::srv::SetCommsLink>::SharedPtr _srvSetComms;

    rclcpp::TimerBase::SharedPtr _timerRx;
    rclcpp::TimerBase::SharedPtr _timerHeartbeat;
    rclcpp::TimerBase::SharedPtr _timerStatusWatchdog;
    rclcpp::TimerBase::SharedPtr _timerImageTx;

    rclcpp::Time _lastPacketReceivedTime;

    // Outgoing image queue on rover
    std::deque<sImageChunkPacket> _outgoingImageChunks;
    uint16_t _txFrameId{1};

    // Incoming frame reassembly on base station
    uint16_t _rxFrameId{0};
    uint8_t _rxTotalChunks{0};
    std::vector<std::vector<uint8_t>> _rxFrameChunks;
    std::vector<bool> _rxChunkReceived;
    size_t _rxChunksCount{0};
    rclcpp::Time _lastChunkTime;
};

#endif  // ROVER_AUXILIARY_SIK_RADIO_NODE_HPP

