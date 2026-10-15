#include "sik_radio_node.hpp"

#include <cmath>
#include <filesystem>
#include <thread>

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
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
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

        _pubEmergencyImage = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            TOPIC_EMERGENCY_IMAGE, QOS_DEFAULT);

        LOG_INFO(Logger::Nodes::SikRadio, "SiK Radio Base Node initialized in Standby. Waiting for GUI activation.");
    }
    else
    {
        // On rover: start publishers
        _pubFallbackJoy = this->create_publisher<rover_msgs::msg::Joy>(TOPIC_FALLBACK_JOY, QOS_DEFAULT);
        _pubHeartbeat = this->create_publisher<std_msgs::msg::Empty>(TOPIC_BASE_HEARTBEAT, QOS_DEFAULT);

        // Emergency snapshot subscriber from rover camera node
        _subEmergencySnapshot = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            TOPIC_EMERGENCY_SNAPSHOT, QOS_DEFAULT,
            [this](const sensor_msgs::msg::CompressedImage& msg_)
            {
                this->queueEmergencySnapshot(msg_);
            });

        // Paced transmission timer for image chunks (35 ms = ~28 chunks/s = ~5 KB/s)
        _timerImageTx = this->create_wall_timer(
            std::chrono::milliseconds(35),
            [this]()
            {
                this->loopSendImageChunk();
            });

        // Listen to mode commands from base station
        _subCommsMode = this->create_subscription<std_msgs::msg::UInt8>(
            TOPIC_COMMS_MODE, QOS_DEFAULT,
            [this](const std_msgs::msg::UInt8& msg_)
            {
                _activeMode = msg_.data;
                if (_activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
                {
                    this->disconnectSerial();
                }
                else
                {
                    this->connectSerial();
                }
            });

        // Rover starts in DORMANT / STANDBY mode with serial port closed until activated
        _activeMode = rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G;
        LOG_INFO(Logger::Nodes::SikRadio, "SiK Radio Rover Node initialized in Standby (Dormant). Waiting for comms mode activation.");
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

std::string SikRadioNode::resolveSerialPort()
{
    // 1. If configured port exists, use it
    if (!_serialPort.empty() && std::filesystem::exists(_serialPort))
    {
        try
        {
            return std::filesystem::canonical(_serialPort).string();
        }
        catch (...)
        {
            return _serialPort;
        }
    }

    // 2. Scan /dev/serial/by-id/ for CP210x, Silicon Labs, FTDI, or SiK devices
    try
    {
        if (std::filesystem::exists("/dev/serial/by-id"))
        {
            for (const auto& entry : std::filesystem::directory_iterator("/dev/serial/by-id"))
            {
                std::string fname = entry.path().filename().string();
                if (fname.find("CP210") != std::string::npos ||
                    fname.find("Silicon_Labs") != std::string::npos ||
                    fname.find("FTDI") != std::string::npos ||
                    fname.find("SiK") != std::string::npos ||
                    fname.find("Holybro") != std::string::npos ||
                    fname.find("3D_Robotics") != std::string::npos ||
                    fname.find("RFD") != std::string::npos)
                {
                    try
                    {
                        std::string canon = std::filesystem::canonical(entry.path()).string();
                        LOG_INFO(Logger::Nodes::SikRadio, "Auto-detected radio: %s -> %s", entry.path().c_str(), canon.c_str());
                        return canon;
                    }
                    catch (...)
                    {
                        LOG_INFO(Logger::Nodes::SikRadio, "Auto-detected radio by-id: %s", entry.path().c_str());
                        return entry.path().string();
                    }
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        LOG_WARN(Logger::Nodes::SikRadio, "Exception scanning /dev/serial/by-id: %s", e.what());
    }

    // 3. Fallback to standard USB serial ports ONLY if configured port is the default /dev/ttyUSB*
    if (_serialPort.empty() || _serialPort == "/dev/ttyUSB0" || _serialPort == "/dev/ttyUSB1")
    {
        if (std::filesystem::exists("/dev/ttyUSB0"))
        {
            LOG_INFO(Logger::Nodes::SikRadio, "Using fallback radio port at /dev/ttyUSB0");
            return "/dev/ttyUSB0";
        }
        if (std::filesystem::exists("/dev/ttyUSB1"))
        {
            LOG_INFO(Logger::Nodes::SikRadio, "Using fallback radio port at /dev/ttyUSB1");
            return "/dev/ttyUSB1";
        }
    }

    return "";
}

bool SikRadioNode::connectSerial(const std::string& port)
{
    if (_serial && _serial->getState() == eState::ACTIVE)
    {
        return true;
    }

    std::string portToOpen = port.empty() ? this->resolveSerialPort() : port;
    if (portToOpen.empty())
    {
        LOG_WARN(Logger::Nodes::SikRadio, "Radio port %s does not exist and no USB serial radio found", _serialPort.c_str());
        return false;
    }

    _activeSerialPort = portToOpen;

    // Retry up to 3 times with 100ms pauses to allow clean driver recovery after disconnect
    // Note: timeout=0, minChar=0 configures non-blocking reads in POSIX termios
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        _serial = std::make_unique<SerialCom>(_activeSerialPort, _baudEnum, eDataPerPacket::EIGHT_BITS, CREAD | CLOCAL, 0, 0);
        if (_serial->getState() == eState::ACTIVE)
        {
            LOG_INFO(Logger::Nodes::SikRadio, "Connected to SiK radio on %s (attempt %d)", _activeSerialPort.c_str(), attempt + 1);
            return true;
        }
        _serial.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_ERROR(Logger::Nodes::SikRadio, "Unable to open radio serial port %s after retries", _activeSerialPort.c_str());
    return false;
}

void SikRadioNode::disconnectSerial()
{
    if (_serial)
    {
        LOG_INFO(Logger::Nodes::SikRadio, "Releasing SiK radio serial port %s", _activeSerialPort.empty() ? _serialPort.c_str() : _activeSerialPort.c_str());
        _serial.reset();
    }
    _activeSerialPort.clear();
    _rxBuffer.clear();
    _outgoingImageChunks.clear();
    _rxFrameChunks.clear();
    _rxChunkReceived.clear();
    _rxChunksCount = 0;
}

uint8_t SikRadioNode::computeImageChunkChecksum(const sImageChunkPacket& pkt)
{
    uint8_t cs = static_cast<uint8_t>(
        (pkt.frame_id & 0xFF) ^
        ((pkt.frame_id >> 8) & 0xFF) ^
        pkt.chunk_idx ^
        pkt.total_chunks ^
        pkt.chunk_len ^
        0x5A);
    for (size_t i = 0; i < pkt.chunk_len && i < sizeof(pkt.data); ++i)
    {
        cs ^= pkt.data[i];
    }
    return cs;
}

void SikRadioNode::queueEmergencySnapshot(const sensor_msgs::msg::CompressedImage& msg_)
{
    if (!_serial || _serial->getState() != eState::ACTIVE || _activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
    {
        return;
    }

    if (msg_.data.empty() || msg_.data.size() > 65000)
    {
        return;
    }

    // Always replace prior unsent chunks so we transmit the most recent snapshot immediately
    _outgoingImageChunks.clear();

    const uint8_t* rawData = msg_.data.data();
    size_t totalSize = msg_.data.size();
    constexpr size_t CHUNK_SIZE = 180;
    uint8_t totalChunks = static_cast<uint8_t>((totalSize + CHUNK_SIZE - 1) / CHUNK_SIZE);

    uint16_t currentFrameId = _txFrameId++;
    if (_txFrameId == 0)
    {
        _txFrameId = 1;
    }

    for (uint8_t idx = 0; idx < totalChunks; ++idx)
    {
        sImageChunkPacket pkt;
        pkt.frame_id = currentFrameId;
        pkt.chunk_idx = idx;
        pkt.total_chunks = totalChunks;

        size_t offset = idx * CHUNK_SIZE;
        size_t len = std::min(CHUNK_SIZE, totalSize - offset);
        pkt.chunk_len = static_cast<uint8_t>(len);
        std::memcpy(pkt.data, rawData + offset, len);
        pkt.checksum = computeImageChunkChecksum(pkt);

        _outgoingImageChunks.push_back(pkt);
    }
}

void SikRadioNode::loopSendImageChunk()
{
    if (_outgoingImageChunks.empty() || !_serial || _serial->getState() != eState::ACTIVE ||
        _activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
    {
        return;
    }

    const sImageChunkPacket& pkt = _outgoingImageChunks.front();
    std::string packetStr(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
    _serial->serialWrite(packetStr);
    _outgoingImageChunks.pop_front();
}

void SikRadioNode::handleImageChunk(const sImageChunkPacket& chunk)
{
    if (chunk.total_chunks == 0 || chunk.chunk_idx >= chunk.total_chunks || chunk.chunk_len > sizeof(chunk.data))
    {
        return;
    }

    // Check if new frame received
    if (chunk.frame_id != _rxFrameId)
    {
        _rxFrameId = chunk.frame_id;
        _rxTotalChunks = chunk.total_chunks;
        _rxFrameChunks.assign(_rxTotalChunks, std::vector<uint8_t>());
        _rxChunkReceived.assign(_rxTotalChunks, false);
        _rxChunksCount = 0;
    }

    if (!_rxChunkReceived[chunk.chunk_idx])
    {
        _rxFrameChunks[chunk.chunk_idx].assign(chunk.data, chunk.data + chunk.chunk_len);
        _rxChunkReceived[chunk.chunk_idx] = true;
        _rxChunksCount++;
        _lastChunkTime = this->now();
    }

    if (_rxChunksCount == _rxTotalChunks && _pubEmergencyImage)
    {
        sensor_msgs::msg::CompressedImage imgMsg;
        imgMsg.header.stamp = this->now();
        imgMsg.header.frame_id = "emergency_camera";
        imgMsg.format = "jpeg";

        size_t totalBytes = 0;
        for (const auto& ch : _rxFrameChunks)
        {
            totalBytes += ch.size();
        }
        imgMsg.data.reserve(totalBytes);
        for (const auto& ch : _rxFrameChunks)
        {
            imgMsg.data.insert(imgMsg.data.end(), ch.begin(), ch.end());
        }

        _pubEmergencyImage->publish(imgMsg);
        LOG_INFO(Logger::Nodes::SikRadio, "Emergency image frame #%u assembled (%zu bytes, %u chunks)",
                 _rxFrameId, totalBytes, _rxTotalChunks);

        // Reset reassembly for next frame
        _rxChunksCount = 0;
        _rxChunkReceived.assign(_rxTotalChunks, false);
    }
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

    auto data = _serial->readRaw();
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

        // 4. Check for Image Chunk packet (Base station side)
        if (_rxBuffer[0] == 0xAA && _rxBuffer[1] == 0x55 && _rxBuffer[2] == 0x03)
        {
            if (_rxBuffer.size() < sizeof(sImageChunkPacket))
            {
                break;
            }
            const sImageChunkPacket* pkt = reinterpret_cast<const sImageChunkPacket*>(_rxBuffer.data());
            uint8_t expectedChecksum = computeImageChunkChecksum(*pkt);

            if (pkt->checksum == expectedChecksum)
            {
                _lastPacketReceivedTime = this->now();
                if (_isBaseStation)
                {
                    this->handleImageChunk(*pkt);
                }
                _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + sizeof(sImageChunkPacket));
                continue;
            }
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
        std::string targetPort = this->resolveSerialPort();
        if (targetPort.empty())
        {
            this->disconnectSerial();
            _activeMode = rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G;
            this->publishSikStatus(0.0f, 0.0f, 0, 0, false);

            std_msgs::msg::UInt8 modeMsg;
            modeMsg.data = _activeMode;
            _pubCommsMode->publish(modeMsg);

            res_->success = false;
            res_->active_mode = _activeMode;
            res_->message = "Failed: No serial radio found (checked " + _serialPort + " and USB serial devices).";
            return;
        }

        if (!this->connectSerial(targetPort))
        {
            this->disconnectSerial();
            _activeMode = rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G;
            this->publishSikStatus(0.0f, 0.0f, 0, 0, false);

            std_msgs::msg::UInt8 modeMsg;
            modeMsg.data = _activeMode;
            _pubCommsMode->publish(modeMsg);

            res_->success = false;
            res_->active_mode = _activeMode;
            res_->message = "Failed: Unable to open radio port " + targetPort;
            return;
        }
        _activeMode = req_->target_mode;
        res_->success = true;
        res_->active_mode = _activeMode;
        res_->message = (_activeMode == rover_msgs::srv::SetCommsLink::Request::MODE_AUTO)
                            ? ("900MHz Auto Failover enabled (" + _activeSerialPort + ").")
                            : ("900MHz Radio link forced active (" + _activeSerialPort + ").");
    }

    LOG_INFO(Logger::Nodes::SikRadio, "Comms mode: %s", res_->message.c_str());

    std_msgs::msg::UInt8 modeMsg;
    modeMsg.data = _activeMode;
    _pubCommsMode->publish(modeMsg);
}
