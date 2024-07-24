#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rover_msgs/msg/gps_position.hpp"
#include "rover_msgs/srv/new_gps_goal.hpp"

#include "rovus_lib/rovus_exceptions.h"
#include "rovus_lib/macros.h"

constexpr char FILE_NAME_ROUTE_DUMP[] = "route_dump.log";

class GoalManager : public rclcpp::Node
{
public:
    GoalManager();
    ~GoalManager() {}

private:
    const std::string _dumpFileName = ament_index_cpp::get_package_share_directory("rover_gui") +
                                      "/../../../../src/rover/rover_gui/saved_files/navigation/" +
                                      FILE_NAME_ROUTE_DUMP;

    rclcpp::Publisher<rover_msgs::msg::GpsPosition>::SharedPtr _pubGpsGoal;
    rclcpp::TimerBase::SharedPtr _timerPub;
    rclcpp::Service<rover_msgs::srv::NewGpsGoal>::SharedPtr _srvNewGoal;

    rover_msgs::msg::GpsPosition _msgCurrentGoal;
    std::vector<rover_msgs::msg::GpsPosition> _route;

    void CB_timerPub(void);
    void sendGoal(void);
    void CB_srvNewGoal(const std::shared_ptr<rover_msgs::srv::NewGpsGoal::Request> request_,
                       std::shared_ptr<rover_msgs::srv::NewGpsGoal::Response> response_);
    void dumpCurrentRouteToFile(void);
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

void GoalManager::CB_srvNewGoal(const std::shared_ptr<rover_msgs::srv::NewGpsGoal::Request> request_,
                                std::shared_ptr<rover_msgs::srv::NewGpsGoal::Response> response_)
{
    response_->success = false;

    switch (request_->type)
    {
    case (rover_msgs::srv::NewGpsGoal::Request::GET_ROUTE):
    {
        if (request_->index != 0 || request_->waypoints.size() != 0)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(0) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            break;
        }

        response_->success = true;
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_ROUTE):
    {
        if (request_->index != 0)
        {
            response_->status = "Index shouldn't be specified when overwritting whole route, no action done.";
            response_->success = false;
            break;
        }

        _route = request_->waypoints;
        response_->success = true;

        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_GOAL_END_APPEND):
    {
        if (request_->index != 0)
        {
            response_->status = "Index should not be specified, No action done";
            response_->success = false;
            break;
        }
        else if (request_->waypoints.size() != 1)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(1) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }

        response_->success = true;
        _route.push_back(request_->waypoints.front());
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_GOAL_END_OVERWRITE):
    {
        if (request_->index != 0)
        {
            response_->status = "Index should not be specified, No action done";
            response_->success = false;
            break;
        }
        else if (request_->waypoints.size() != 1)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(1) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }
        else if (_route.size() == 0)
        {
            response_->status = "Route is empty. Can't replace current end goal, try to use type:NEW_GOAL_END_APPEND instead";
            response_->success = false;
            break;
        }

        response_->success = true;
        _route.back() = request_->waypoints.front();
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_WAYPOINT_BEFORE_END):
    {
        if (request_->index != 0)
        {
            response_->status = "Index should not be specified, No action done";
            response_->success = false;
            break;
        }
        else if (request_->waypoints.size() != 1)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(1) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }

        if (_route.size() > 0)
        {
            _route.insert(_route.end() - 1, request_->waypoints.front());
        }
        else
        {
            _route.push_back(request_->waypoints.front());
        }

        response_->success = true;

        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_WAYPOINT_INDEX_INSERT):
    {
        if (request_->waypoints.size() != 1)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(1) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }

        if (request_->index >= 1 && (int8_t)request_->index > ((int8_t)_route.size() - 1))
        {
            response_->status = "Invalid index value, index not in route";
            response_->success = false;
            break;
        }

        response_->success = true;
        _route.insert(_route.begin() + request_->index, request_->waypoints.front());
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::NEW_WAYPOINT_INDEX_REPLACE):
    {
        if (request_->waypoints.size() != 1)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(1) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }

        if ((int8_t)request_->index > ((int8_t)_route.size() - 1))
        {
            response_->status = "Invalid index value, index not in route";
            response_->success = false;
            break;
        }

        if (_route.size() == 0)
        {
            response_->status = "Invalid index value, index not in route, route is empty";
            response_->success = false;
            break;
        }

        response_->success = true;
        _route[request_->index] = request_->waypoints[0];
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::CLEAR_WAYPOINT_INDEX):
    {
        if (request_->waypoints.size() != 0)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(0) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }

        if ((int8_t)request_->index > ((int8_t)_route.size() - 1))
        {
            response_->status = "Invalid index value, index not in route";
            response_->success = false;
            break;
        }

        response_->success = true;
        _route.erase(_route.begin() + request_->index);
        break;
    }

    case (rover_msgs::srv::NewGpsGoal::Request::CLEAR_ROUTE):
    {
        if (request_->waypoints.size() != 0)
        {
            response_->status = "Receive new waypoints incoherent with request, expected " + std::to_string(0) +
                                " waypoints, received " + std::to_string(request_->waypoints.size());
            response_->success = false;
            break;
        }

        if (request_->index != 0)
        {
            response_->status = "Invalid index value, should be 0";
            response_->success = false;
            break;
        }

        response_->success = true;
        _route.clear();
        break;
    }

    default:
    {
        response_->success = false;
        response_->status = "Invalid or unknowed request " + std::to_string(request_->type) + ", no action done.";
    }
    }

    if (response_->status != "")
    {
        std::string waypointStr;
        char buffer[255];
        for (size_t i = 0; i < request_->waypoints.size(); i++)
        {
            std::snprintf(buffer, sizeof(buffer), " (lat:\"%.5f, long:%.5f) ",
                          request_->waypoints[i].latitude,
                          request_->waypoints[i].longitude);
            waypointStr += buffer;
        }

        RCLCPP_WARN(this->get_logger(), ("Bad request of type:\"" + std::to_string(request_->type) +
                                         "\", with specified index:\"" + std::to_string(request_->index) +
                                         "\", and with waypoints:[\"" + waypointStr + "]\" -> " + response_->status)
                                            .c_str());
    }

    response_->route = _route;
    this->sendGoal();

    if (_route.size() > 0)
    {
        this->dumpCurrentRouteToFile();
    }
}

void GoalManager::dumpCurrentRouteToFile(void)
{
    FILE *file = fopen(_dumpFileName.c_str(), "a");
    if (!file)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to dump updated route to file... Make sure following folder structure exist: %s", _dumpFileName.c_str());
    }
    else
    {
        fprintf(file, "BEGIN_%lu_%lu", std::time(NULL), _route.size());

        for (uint8_t i = 0; i < _route.size(); i++)
        {
            fprintf(file, "_{%f_%f}", _route[i].latitude, _route[i].longitude);
        }
        fprintf(file, "_END\n");
    }
    fclose(file);
}
