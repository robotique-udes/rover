#ifndef GOAL_MANAGER_NODE_HPP
#define GOAL_MANAGER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps.hpp"
#include <rover_msgs/msg/propulsion_motor.hpp>
#include "rover_msgs/srv/desired_gps_position.hpp"

#include "navigation_controller.hpp"

class GoalManager : public rclcpp::Node
{
    static constexpr char* TOPIC_GPS_NAME = "/rover/gps/position";
    static constexpr char* SRV_GOAL_NAME = "/rover/goal/position";
    static constexpr char* TOPIC_WHEEL_CMD_NAME = "/rover/drive_train/wheels_cmd_auto";
    static constexpr uint64_t PUBLISHER_PERIOD_MS = 200UL;
    static constexpr float HEADING_BUFFER = 1.0F;

  public:
    enum class eState
    {
        IDLE = 0,
        ROTATING = 1,
        NAVIGATING_TO_POINT = 2,
        ERROR
    };
    GoalManager();
    ~GoalManager() = default;

  private:
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_currentGps;
    rclcpp::Service<rover_msgs::srv::DesiredGpsPosition>::SharedPtr _srv_desiredGps;
    rclcpp::Publisher<rover_msgs::msg::PropulsionMotor>::SharedPtr _pub_auto_cmd;
    rclcpp::TimerBase::SharedPtr _timer;

    NavigationController _navigationController;
    eState _state = eState::IDLE;

    bool goalRequested = false;
    bool goalReached = false;
    bool _desiredHeadingReached = false;

    std::array<float, TO_UNDERLYING(NavigationController::eWheelCmd::eLAST)> _targetWheelCmd = {0.0F, 0.0F, 0.0F, 0.0F};

  public:
    void CB_currentGps(const rover_msgs::msg::Gps& gpsMsg_);
    void CB_desiredGps(const rover_msgs::srv::DesiredGpsPosition::Request::SharedPtr request_,
                       rover_msgs::srv::DesiredGpsPosition::Response::SharedPtr response_);
    void driveTrainPublisher(void);
    bool desiredHeadingReached(NavigationController::eRotationDirection rotationDirection_);
};

#endif