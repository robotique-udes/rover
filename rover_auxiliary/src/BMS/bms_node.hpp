#ifndef BMS_NODE_HPP
#define BMS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_msgs/msg/bms_data.hpp"
#include <utility>
#include <fstream>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include "rover_lib2/communication/Serial/serial_com.hpp"

DEFINE_LOG_NODE(BmsInfo, Logger::eNodeState::ON);

class BMSDataNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_BMS_DATA = "rover/auxiliary/bms_data";
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1000UL;
    static constexpr uint16_t MAX_CELL = 6;

  public:
    BMSDataNode();
    void callbackBMSData(void);
    void getData(void);
    void parse(std::string rawOutput, uint16_t dataNumber, std::vector<uint16_t>& dataArray);

  private:
    double _batteryAmps;
    uint16_t _batteryVolt;
    std::vector<uint16_t> _cellVolt;
    rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
};

#endif