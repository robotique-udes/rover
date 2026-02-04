#ifndef BMS_NODE_HPP
#define BMS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_msgs/msg/bms_data.hpp"
#include <utility>
#include <fstream>
#include <iostream>


class BMSDataNode : public rclcpp::Node
{

    static constexpr const char* TOPIC_BMS_DATA = "rover/auxiliary/bms_data";
    static constexpr const char* DATA_FILE_PATH = "rover_auxiliary/src/BMS/script/bms_data_log.txt";
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1000UL;
    static constexpr uint8_t MAX_CELL = 6;


    public:

        BMSDataNode(int argc, char** argv);
        void callbackBMSData();
        int getData();

    private:

        uint8_t _batteryAmps;
        uint8_t _batterySOC;
        uint8_t _batteryVolt;
        uint8_t _cellVolt[MAX_CELL] = {0};
        rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
        rclcpp::TimerBase::SharedPtr _timer_publisher;
};







#endif