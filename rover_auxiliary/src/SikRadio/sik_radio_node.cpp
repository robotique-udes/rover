#include "sik_radio_node.hpp"

#include <cmath>
#include <filesystem>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<SikRadioNode> node = std::make_shared<SikRadioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

SikRadioNode::SikRadioNode() : Node("sik_radio_node")
{
    this->initNode();
}

void SikRadioNode::initNode()
{
    this->declare_parameter<bool>("is_base", false);
    this->declare_parameter<std::string>("serial_port", "/dev/ttySikRadio");
    this->declare_parameter<int>("baud_rate", 115200);

    _isBaseStation = this->get_parameter("is_base").as_bool();
    _serialPort = this->get_parameter("serial_port").as_string();
    const int baudInt = this->get_parameter("baud_rate").as_int();

    _baudEnum = (baudInt == 57600) ? eBaudRate::B_57600 : eBaudRate::B_115200;

    _pubSikStatus = this->create_publisher<rover_msgs::msg::SikStatus>(TOPIC_SIK_STATUS, QOS_DEFAULT);
    _pubCommsMode = this->create_publisher<std_msgs::msg::UInt8>(TOPIC_COMMS_MODE, QOS_DEFAULT);

    _lastPacketReceivedTime = this->now();

    if (_isBaseStation)
    {
        // On base station: start in DORMANT (PRIMARY_5G) mode. Serial port is NOT opened until user activates it in GUI.
        _activeMode = rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G;

        _subJoy = this->create_subscription<rover_msgs::msg::Joy>(
            TOPIC_JOY_DRIVE, QOS_DEFAULT,
            [this](const rover_msgs::msg::Joy& msg_)
            {
                this->sendJoyPacket(msg_);
            });

        _subHeartbeat = this->create_subscription<std_msgs::msg::Empty>(
            TOPIC_BASE_HEARTBEAT, QOS_DEFAULT,
            [this](const std_msgs::msg::Empty& /*msg_*/)
            {
                this->sendHeartbeatPacket();
            });

        _srvSetComms = this->create_service<rover_msgs::srv::SetCommsLink>(
            SERVICE_SET_COMMS,
            [this](const std::shared_ptr<rover_msgs::srv::SetCommsLink::Request> req_,
                   std::shared_ptr<rover_msgs::srv::SetCommsLink::Response> res_)
            {
                this->cbSetComms(req_, res_);
            });

        LOG_INFO(Logger::Nodes::SikRadio, "SiK Radio Base Node initialized in Standby. Waiting for GUI activation.");
    }
    else
    {
        // On rover: start publishers
        _pubFallbackJoy = this->create_publisher<rover_msgs::msg::Joy>(TOPIC_FALLBACK_JOY, QOS_DEFAULT);
        _pubHeartbeat = this->create_publisher<std_msgs::msg::Empty>(TOPIC_BASE_HEARTBEAT, QOS_DEFAULT);

        // Listen to mode commands from base station
        _subCommsMode = this->create_subscription<std_msgs::msg::UInt8>(
            TOPIC_COMMS_MODE, QOS_DEFAULT,
            [this](const std_msgs::msg::UInt8& msg_)
            {
                if (msg_.data == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
                {
                    this->disconnectSerial();
                }
                else
                {
                    this->connectSerial();
                }
            });

        // If port exists on rover at boot, open it; otherwise stay dormant without failing
        if (std::filesystem::exists(_serialPort))
        {
            this->connectSerial();
        }
        else
        {
            LOG_INFO(Logger::Nodes::SikRadio, "SiK Radio port %s not found on rover. Remaining dormant.", _serialPort.c_str());
        }
    }

    // Polling timer for serial reading
    _timerRx = this->create_wall_timer(std::chrono::milliseconds(20),
                                       [this]()
                                       {
                                           this->loopReceive();
                                       });

    // Connection watchdog timer
    _timerStatusWatchdog = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                   [this]()
                                                   {
                                                       if (_activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
                                                       {
                                                           this->publishSikStatus(0.0f, 0.0f, 0, 0, false);
                                                       }
                                                       else if ((this->now() - _lastPacketReceivedTime).seconds() > 3.0)
                                                       {
                                                           this->publishSikStatus(0.0f, 0.0f, 0, 0, false);
                                                       }
                                                   });
}

bool SikRadioNode::connectSerial()
{
    if (_serial && _serial->getState() == eState::ACTIVE)
    {
        return true;
    }

    if (!std::filesystem::exists(_serialPort))
    {
        LOG_WARN(Logger::Nodes::SikRadio, "Radio port %s does not exist on filesystem", _serialPort.c_str());
        return false;
    }

    _serial = std::make_unique<SerialCom>(_serialPort, _baudEnum);
    if (_serial->getState() != eState::ACTIVE)
    {
        LOG_ERROR(Logger::Nodes::SikRadio, "Unable to open radio serial port %s", _serialPort.c_str());
        _serial.reset();
        return false;
    }

    LOG_INFO(Logger::Nodes::SikRadio, "Connected to SiK radio on %s", _serialPort.c_str());
    return true;
}

void SikRadioNode::disconnectSerial()
{
    if (_serial)
    {
        LOG_INFO(Logger::Nodes::SikRadio, "Releasing SiK radio serial port %s", _serialPort.c_str());
        _serial.reset();
    }
    _rxBuffer.clear();
}

void SikRadioNode::sendJoyPacket(const rover_msgs::msg::Joy& msg_)
{
    if (!_serial || _activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
    {
        return;
    }

    sCompactJoyPacket pkt;
    float linearVal = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::LINEAR_INPUT)];
    float angularVal = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::ANGULAR_INPUT)];
    float deadmanVal = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::DEADMAN_SWITCH)];
    float turboVal = msg_.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::MODE_TURBO_ENABLE)];

    pkt.linear = static_cast<int16_t>(std::clamp(linearVal, -1.0f, 1.0f) * 1000.0f);
    pkt.angular = static_cast<int16_t>(std::clamp(angularVal, -1.0f, 1.0f) * 1000.0f);
    pkt.deadman = (deadmanVal > 0.5f) ? 1 : 0;
    pkt.mode_turbo = (turboVal > 0.5f) ? 1 : 0;

    pkt.checksum = static_cast<uint8_t>(pkt.linear ^ pkt.angular ^ pkt.deadman ^ pkt.mode_turbo ^ 0xA5);

    std::string packetStr(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
    _serial->serialWrite(packetStr);
}

void SikRadioNode::sendHeartbeatPacket()
{
    if (!_serial || _activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
    {
        return;
    }

    sHeartbeatPacket pkt;
    std::string packetStr(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
    _serial->serialWrite(packetStr);
}

void SikRadioNode::loopReceive()
{
    if (!_serial || _serial->getState() != eState::ACTIVE)
    {
        return;
    }

    auto data = _serial->serialRead();
    if (!data.has_value() || data->empty())
    {
        return;
    }

    const uint8_t* rawData = reinterpret_cast<const uint8_t*>(data->data());
    _rxBuffer.insert(_rxBuffer.end(), rawData, rawData + data->size());

    if (_rxBuffer.size() > 1024)
    {
        _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + (_rxBuffer.size() - 512));
    }

    this->parseBuffer();
}

void SikRadioNode::parseBuffer()
{
    while (_rxBuffer.size() >= 4)
    {
        // 1. Check for MAVLink v1 RADIO_STATUS (ID 109 = 0x6D)
        if (_rxBuffer[0] == 0xFE && _rxBuffer.size() >= 17)
        {
            uint8_t payloadLen = _rxBuffer[1];
            uint8_t msgId = _rxBuffer[5];
            if (msgId == 109 && payloadLen >= sizeof(sMavlinkRadioStatusV1))
            {
                const sMavlinkRadioStatusV1* status = reinterpret_cast<const sMavlinkRadioStatusV1*>(&_rxBuffer[6]);
                float localRssi = (status->rssi > 0) ? (static_cast<float>(status->rssi) / 1.9f - 127.0f) : -120.0f;
                float remRssi = (status->remrssi > 0) ? (static_cast<float>(status->remrssi) / 1.9f - 127.0f) : -120.0f;

                this->publishSikStatus(localRssi, remRssi, status->txbuf, status->noise, true);
                _lastPacketReceivedTime = this->now();

                size_t packetSize = 6 + payloadLen + 2;
                if (_rxBuffer.size() >= packetSize)
                {
                    _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + packetSize);
                    continue;
                }
            }
        }

        // 2. Check for our Compact Joy packet (Rover side)
        if (_rxBuffer[0] == 0xAA && _rxBuffer[1] == 0x55 && _rxBuffer[2] == 0x01)
        {
            if (_rxBuffer.size() < sizeof(sCompactJoyPacket))
            {
                break;
            }
            const sCompactJoyPacket* pkt = reinterpret_cast<const sCompactJoyPacket*>(_rxBuffer.data());
            uint8_t expectedChecksum = static_cast<uint8_t>(pkt->linear ^ pkt->angular ^ pkt->deadman ^ pkt->mode_turbo ^ 0xA5);

            if (pkt->checksum == expectedChecksum)
            {
                _lastPacketReceivedTime = this->now();
                if (!_isBaseStation && _pubFallbackJoy)
                {
                    rover_msgs::msg::Joy joyMsg;
                    joyMsg.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::LINEAR_INPUT)] = static_cast<float>(pkt->linear) / 1000.0f;
                    joyMsg.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::ANGULAR_INPUT)] = static_cast<float>(pkt->angular) / 1000.0f;
                    joyMsg.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::DEADMAN_SWITCH)] = static_cast<float>(pkt->deadman);
                    joyMsg.joy_data[std::to_underlying(Constants::Keybinds::DriveTrain::MODE_TURBO_ENABLE)] = static_cast<float>(pkt->mode_turbo);
                    _pubFallbackJoy->publish(joyMsg);
                }
                _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + sizeof(sCompactJoyPacket));
                continue;
            }
        }

        // 3. Check for Heartbeat packet (Rover side)
        if (_rxBuffer[0] == 0xAA && _rxBuffer[1] == 0x55 && _rxBuffer[2] == 0x02)
        {
            if (_rxBuffer.size() < sizeof(sHeartbeatPacket))
            {
                break;
            }
            _lastPacketReceivedTime = this->now();
            if (!_isBaseStation && _pubHeartbeat)
            {
                std_msgs::msg::Empty hbMsg;
                _pubHeartbeat->publish(hbMsg);
            }
            _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + sizeof(sHeartbeatPacket));
            continue;
        }

        // Drop leading byte if no header matched
        _rxBuffer.erase(_rxBuffer.begin());
    }
}

void SikRadioNode::publishSikStatus(float rssi_, float remRssi_, uint8_t txbuf_, uint8_t noise_, bool connected_)
{
    rover_msgs::msg::SikStatus msg;
    msg.connected = connected_;
    msg.rssi = rssi_;
    msg.remote_rssi = remRssi_;
    msg.txbuf = txbuf_;
    msg.noise = noise_;
    _pubSikStatus->publish(msg);
}

void SikRadioNode::cbSetComms(const std::shared_ptr<rover_msgs::srv::SetCommsLink::Request> req_,
                             std::shared_ptr<rover_msgs::srv::SetCommsLink::Response> res_)
{
    if (req_->target_mode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
    {
        this->disconnectSerial();
        _activeMode = rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G;
        this->publishSikStatus(0.0f, 0.0f, 0, 0, false);
        res_->success = true;
        res_->active_mode = _activeMode;
        res_->message = "900MHz Radio disabled (Primary link active).";
    }
    else
    {
        if (!this->connectSerial())
        {
            res_->success = false;
            res_->active_mode = _activeMode;
            res_->message = "Failed: Radio port " + _serialPort + " not found or could not be opened.";
            return;
        }
        _activeMode = req_->target_mode;
        res_->success = true;
        res_->active_mode = _activeMode;
        res_->message = (_activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_AUTO)
                            ? "900MHz Auto Failover enabled."
                            : "900MHz Radio link forced active.";
    }

    LOG_INFO(Logger::Nodes::SikRadio, "Comms mode: %s", res_->message.c_str());

    std_msgs::msg::UInt8 modeMsg;
    modeMsg.data = _activeMode;
    _pubCommsMode->publish(modeMsg);
}
