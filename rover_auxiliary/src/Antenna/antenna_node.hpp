#ifndef ANTENNA_NODE_HPP
#define ANTENNA_NODE_HPP
#include "antenna_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/antenna_status.hpp>

class AntennaNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_ANTENNA_STATUS = "/rover/antenna/status";
    static constexpr const char* ANTENNA_BASE_USERNAME = "ROVER_ANTENNA_BASE_USERNAME";
    static constexpr const char* ANTENNA_BASE_PASSWORD = "ROVER_ANTENNA_BASE_PASSWORD";
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 250UL;

  public:
    AntennaNode();

  private:
    /**
     * @brief Loads the username and password variables from environment varibles
     */
    bool loadUserInfo(void);
    void retrieveDriverInfosAndPublish(void);

    std::unique_ptr<AntennaDriver> _driver;

    rclcpp::Publisher<rover_msgs::msg::AntennaStatus>::SharedPtr _pub_antennaStatus;
    rclcpp::TimerBase::SharedPtr _timer_pubAntennaStatus;
    std::string _username;
    std::string _password;
};

#endif  // ANTENNA_NODE_HPP