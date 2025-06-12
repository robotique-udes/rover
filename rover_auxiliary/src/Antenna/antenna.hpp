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
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 5000UL;

  public:
    // Constructor initializes curl once
    AntennaNode();

    // Destructor cleans up curl
    ~AntennaNode();

    AntennaNode(const AntennaNode&) = delete;
    AntennaNode& operator=(const AntennaNode&) = delete;

  private:
    rclcpp::Publisher<rover_msgs::msg::AntennaStatus>::SharedPtr _pub_antenna_status;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    void CB_antenna_publisher(void);

    std::string _AntennaCookie;


    //placeholders
    std::string username = "placeholder";
    std::string password = "placeholder";
};

#endif  // ANTENNA_NODE_HPP