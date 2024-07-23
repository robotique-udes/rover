#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/gps_position.hpp"
#include "rover_msgs/srv/new_gps_goal.hpp"

#include "rovus_lib/rovus_exceptions.h"
#include "rovus_lib/macros.h"

#warning must dump each changes as new route in backup file

class GoalManager : public rclcpp::Node
{
public:
    GoalManager();
    ~GoalManager() {}

private:
    rclcpp::Publisher<rover_msgs::msg::GpsPosition>::SharedPtr _pubGpsGoal;
    rclcpp::TimerBase::SharedPtr _timerPub;
    rclcpp::Service<rover_msgs::srv::NewGpsGoal>::SharedPtr _srvNewGoal;

    rover_msgs::msg::GpsPosition _msgCurrentGoal;
    std::vector<rover_msgs::msg::GpsPosition> _route;

    void CB_timerPub(void);
    void sendGoal(void);
    void CB_srvNewGoal(const std::shared_ptr<rover_msgs::srv::NewGpsGoal::Request> request,
                       std::shared_ptr<rover_msgs::srv::NewGpsGoal::Response> response);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalManager>());
    rclcpp::shutdown();
    return 0;
}

GoalManager::GoalManager() : Node("goal_manager")
{
    _pubGpsGoal = this->create_publisher<rover_msgs::msg::GpsPosition>("/rover/drive_train/auto/gps/goal", 1);
    _timerPub = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GoalManager::CB_timerPub, this));
    _srvNewGoal = this->create_service<rover_msgs::srv::NewGpsGoal>("/rover/drive_train/auto/manage_goal",
                                                                    std::bind(&GoalManager::CB_srvNewGoal,
                                                                              this,
                                                                              std::placeholders::_1,
                                                                              std::placeholders::_2));
}

void GoalManager::CB_timerPub()
{
    sendGoal();
}

void GoalManager::sendGoal(void)
{
    _pubGpsGoal->publish(_msgCurrentGoal);
}

void GoalManager::CB_srvNewGoal(const std::shared_ptr<rover_msgs::srv::NewGpsGoal::Request> request,
                                std::shared_ptr<rover_msgs::srv::NewGpsGoal::Response> response)
{
    response->success = false;

    switch (request->type)
    {
    case (rover_msgs::srv::NewGpsGoal::Request::GET_ROUTE):
    {
        response->route = _route;
        response->success = true;

        if(request->index != 0 && request->waypoint.size() != 0)
        {
            response->status = "Receive new waypoint incoherent with request, no action done.";
        }
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_ROUTE):
    {
        response->route = _route;
        response->success = true;
        break;
    }

    default:
    {
        response->success = false;
        response->status = "Invalid or unknowed request " + std::to_string(request->type) + ", no action done.";
    }
    }

    // if (request->index == rover_msgs::srv::LightControl::Request::LIGHT)
    // {
    //     RCLCPP_INFO(LOGGER, request->enable == true ? "Light enabled": "Light disabled");
    //     _msgLights.enable[rover_msgs::msg::LightControl::LIGHT] = request->enable;
    //     response->success = true;
    // }

    // if (request->index == rover_msgs::srv::LightControl::Request::LIGHT_INFRARED)
    // {
    //     RCLCPP_INFO(LOGGER, request->enable == true ? "Infrared light enabled": "Infrared light disabled");
    //     _msgLights.enable[rover_msgs::msg::LightControl::LIGHT_INFRARED] = request->enable;
    //     response->success = true;
    // }

    // if (response->success == false)
    // {
    //     RCLCPP_WARN(LOGGER, "Error when processing call, no action done");
    // }

    this->sendGoal();
}
