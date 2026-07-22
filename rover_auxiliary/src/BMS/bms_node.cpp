#include "bms_node.hpp"

int main(int argc_, char** argv_)
{
    rclcpp::init(argc_, argv_);
    std::shared_ptr<BMSDataNode> node = std::make_shared<BMSDataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

BMSDataNode::BMSDataNode():
    Node("bms_info"),
    _terminal(DEVICE_FILE_PATH)
{
    _publisher = create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    _timer_publisher = create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                         [this](void)
                                         {
                                             this->callbackBMSData();
                                         });
}

void BMSDataNode::callbackBMSData(void)
{
    rover_msgs::msg::BmsData msg;

    if (this->getData())
    {
        msg.valid = true;
        msg.battery_amps = _ampArray[std::to_underlying(AmpIndexType::BATTERY_AMPS)];
        msg.cell_volt.resize(std::to_underlying(VoltIndexType::CELL_VOLT_END)
                             - std::to_underlying(VoltIndexType::CELL_VOLT_START));

        for (size_t i = std::to_underlying(VoltIndexType::CELL_VOLT_START); i < std::to_underlying(VoltIndexType::CELL_VOLT_END);
             i++)
        {
            msg.cell_volt[i - std::to_underlying(VoltIndexType::CELL_VOLT_START)] = _voltArray[i];
        }
        _failedAttempts = 0;
    }
    else
    {
        msg.valid = false;
        _failedAttempts++;
        _terminal.flushInput();

        if (_failedAttempts > MAX_FAILED_ATTEMPTS)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 LOGGER_THROTTLE_MS,
                                 "Consecutive failures, reconnecting serial port");
            _terminal.reconnect();
            _failedAttempts = 0;
        }
    }

    _publisher->publish(msg);
}

std::optional<std::string> BMSDataNode::readDataFrame()
{
    for (int attempt = 0; attempt < MAX_ECHO_SKIPS; ++attempt)
    {
        std::optional<std::string> frame = _terminal.serialRead();
        if (!frame)
        {
            return std::nullopt;
        }
        if (!frame->empty() && frame->front() == '?')
        {
            continue;
        }
        return frame;
    }
    return std::nullopt;
}

bool BMSDataNode::getData(void)
{
    if (!_terminal.serialWrite("?A\r"))
    {
        return false;
    }

    const std::optional<std::string> ampResult = this->readDataFrame();
    if (!ampResult)
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve amp data failed");
        return false;
    }
    if (!this->parse(ampResult.value(), "A", _ampArray))
    {
        return false;
    }

    if (!_terminal.serialWrite("?V\r"))
    {
        return false;
    }

    const std::optional<std::string> voltResult = this->readDataFrame();
    if (!voltResult)
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve volt data failed");
        return false;
    }
    if (!this->parse(voltResult.value(), "V", _voltArray))
    {
        return false;
    }

    return true;
}

template<size_t N>
bool BMSDataNode::parse(std::string_view view_, std::string_view expectedPrefix_, std::array<uint16_t, N>& dataArray_)
{
    std::array<uint16_t, N> tempArray{};
    const size_t startPos = view_.find('=');
    if (startPos == std::string_view::npos)
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve data from BMS failed");
        return false;
    }

    if (view_.substr(0, startPos) != expectedPrefix_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Wrong response type. Expected '%.*s', got '%.*s'",
                    static_cast<int>(expectedPrefix_.size()),
                    expectedPrefix_.data(),
                    static_cast<int>(startPos),
                    view_.data());
        return false;
    }

    view_.remove_prefix(startPos + 1);
    for (size_t i = 0; i < dataArray_.size(); i++)
    {
        size_t delimPos = view_.find(':');

        size_t end;
        if (delimPos == std::string_view::npos)
        {
            end = view_.size();
            if (i < dataArray_.size() - 1)
            {
                RCLCPP_WARN(this->get_logger(), "BMS's command output returned fewer data than expected");
            }
        }
        else
        {
            end = delimPos;
        }

        uint16_t value;
        std::from_chars_result charResult = std::from_chars(view_.data(), view_.data() + end, value);
        if (charResult.ec == std::errc{} && charResult.ptr == view_.data() + end)
        {
            tempArray[i] = value;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to parse BMS value");
            return false;
        }

        if (delimPos != std::string_view::npos)
        {
            view_.remove_prefix(delimPos + 1);
            if (i == dataArray_.size() - 1)
            {
                RCLCPP_WARN(this->get_logger(),
                            "More data than expected. Remainder: %.*s",
                            static_cast<int>(view_.size()),
                            view_.data());
            }
        }
        else if (i < dataArray_.size() - 1)
        {
            return false;
        }
    }
    dataArray_ = tempArray;

    return true;
}