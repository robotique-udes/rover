#ifndef ANTENNA_NODE_HPP
#define ANTENNA_NODE_HPP

#include <string>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/antenna_status.hpp>
#include <cpr/cpr.h>

class AntennaNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_ANTENNA_STATUS = "/rover/antenna/status";
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 250UL;

  public:
    // Constructor initializes curl once
    AntennaNode();

    // Destructor cleans up curl
    ~AntennaNode();

    AntennaNode(const AntennaNode&) = delete;
    AntennaNode& operator=(const AntennaNode&) = delete;

  private:
    void CB_antenna_publisher(void);
    bool login(void);
    bool getIfStats(rover_msgs::msg::AntennaStatus* msg_);
    bool getStatus(rover_msgs::msg::AntennaStatus* msg_);

    rclcpp::Publisher<rover_msgs::msg::AntennaStatus>::SharedPtr _pub_antenna_status;
    rclcpp::TimerBase::SharedPtr _timer_pub;
    std::shared_ptr<cpr::Session> _session = nullptr;
    bool _is_logged_in = false;
    uint64_t _lanRxBytes = 0;
    uint64_t _lanTxBytes = 0;
    uint64_t _wlanRxBytes = 0;
    uint64_t _wlanTxBytes = 0;

    // placeholders
    std::string username = "placeholder";
    std::string password = "placeholder";
};

#endif  // ANTENNA_NODE_HPP