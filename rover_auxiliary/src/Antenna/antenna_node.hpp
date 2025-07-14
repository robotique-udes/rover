#ifndef ANTENNA_NODE_HPP
#define ANTENNA_NODE_HPP
#include "antenna_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/antenna_status.hpp>

class AntennaNode : public rclcpp::Node
{
    static constexpr const char* ENV_PATH = "/ros2_ws/src/rover/.env";
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 250UL;

  public:
    AntennaNode();

  private:
    bool loadEnvFile(void);

    rclcpp::Publisher<rover_msgs::msg::AntennaStatus>::SharedPtr _pubAntennaStatus;
    rclcpp::TimerBase::SharedPtr _timer_pub;
    std::string _username;
    std::string _password;
};




#endif // ANTENNA_NODE_HPP