#ifndef ANTENNA_NODE_HPP
#define ANTENNA_NODE_HPP
#include "antenna_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/antenna_status.hpp>

class AntennaNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_ANTENNA_STATUS = "/rover/antenna/status";
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 250UL;

  public:
    AntennaNode();

  private:
    /**
     * @brief Loads the username and password variables from the .env file in /rover/
     *
     * @warning you need to add the fields "username" and "password" to your .env
     */
    bool loadUserInfo(void);
    void executeDriver(void);

    AntennaDriver _driver;

    rclcpp::Publisher<rover_msgs::msg::AntennaStatus>::SharedPtr _pub_antennaStatus;
    rclcpp::TimerBase::SharedPtr _timer_pub;
    std::string _username;
    std::string _password;
};

#endif  // ANTENNA_NODE_HPP