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

    this->getData();

    msg.battery_amps = _ampArray[static_cast<size_t>(AmpIndexType::BATTERY_AMPS)];

    for (size_t i = static_cast<size_t>(VoltIndexType::CELL_VOLT_START); i < static_cast<size_t>(VoltIndexType::CELL_VOLT_END);
         i++)
    {
        msg.cell_volt[i - static_cast<size_t>(VoltIndexType::CELL_VOLT_START)] = _voltArray[i];
    }

    _publisher->publish(msg);
}

void BMSDataNode::getData(void)
{
    _terminal.serialWrite("?A\r");
    auto ampResult = _terminal.serialRead();

    if (ampResult.has_value())
    {
        this->parse(ampResult.value(), _ampArray);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve amp data failed");
    }

    _terminal.serialWrite("?V\r");
    auto voltResult = _terminal.serialRead();
    if (voltResult.has_value())
    {
        this->parse(voltResult.value(), _voltArray);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve volt data failed");
    }
}

template<size_t N>
void BMSDataNode::parse(std::string_view view_, std::array<uint16_t, N>& dataArray_)
{
    size_t startPos = view_.find('=');
    if (startPos == std::string_view::npos)
    {
        RCLCPP_WARN(this->get_logger(), "Command to retrieve data from BMS failed");
        return;
    }

    view_.remove_prefix(startPos + 1);

    if (view_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No data retrieved from BMS command");
        return;
    }

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
        if (charResult.ec == std::errc{})
        {
            dataArray_.at(i) = value;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to parse BMS value");
            return;
        }

        if (delimPos != std::string_view::npos)
        {
            view_.remove_prefix(delimPos + 1);
        }
    }
}