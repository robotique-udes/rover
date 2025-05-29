#ifndef GOAL_MANAGER_NODE_HPP
#define GOAL_MANAGER_NODE_HPP

#include "navigation_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"
#include <rover_msgs/msg/propulsion_motor.hpp>

class GoalManager : public rclcpp::Node
{
    static constexpr char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr char* TOPIC_GOAL_NAME = "/rover/goal/position";
    static constexpr char* TOPIC_WHEEL_CMD_NAME = "/rover/drive_train/wheels_cmd_auto";

  public:
    GoalManager();
    ~GoalManager() = default;

    private:
        rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_currentGps;
        rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_DesiredGps;
        rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_auto_cmd;

        NavigationController _navigationController;
};

#endif