#ifndef BMS_NODE_HPP
#define BMS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include "rover_msgs/msg/bms_data.hpp"
#include <utility>

class BMSDataNode : public rclcpp::Node
{

    static constexpr const char* TOPIC_BMS_DATA = "rover/auxiliary/bms_data";
    static constexpr uint64_t DELAY_PUBLISHER_MS = 1000UL;

    public:

        BMSDataNode(int argc, char** argv);
        void callbackBMSData();

    private:

    rclcpp::Publisher<rover_msgs::msg::BmsData>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_publisher;
};







#endif