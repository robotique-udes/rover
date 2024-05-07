#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/light_control.hpp"
#include "rover_msgs/srv/light_control.hpp"

#include "rovus_lib/rovus_exceptions.h"
#include "rovus_lib/macros.h"

class LightControl : public rclcpp::Node
{
public:
    LightControl();
    ~LightControl() {}

private:
    rclcpp::Publisher<rover_msgs::msg::LightControl>::SharedPtr _pubLights;
    rclcpp::TimerBase::SharedPtr _timerPub;
    rclcpp::Service<rover_msgs::srv::LightControl>::SharedPtr _srvLightControl;

    rover_msgs::msg::LightControl _msgLights;
    void CB_timer(void);
    void sendCmd(void);
    void CB_srv(const std::shared_ptr<rover_msgs::srv::LightControl::Request> request,
                std::shared_ptr<rover_msgs::srv::LightControl::Response> response);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightControl>());
    rclcpp::shutdown();
    return 0;
}

LightControl::LightControl() : Node("light_controller")
{
    _pubLights = this->create_publisher<rover_msgs::msg::LightControl>("/rover/auxiliary/lights/status", 1);
    _timerPub = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LightControl::CB_timer, this));
    _srvLightControl = this->create_service<rover_msgs::srv::LightControl>("/rover/auxiliary/set/lights", std::bind(&LightControl::CB_srv, this, std::placeholders::_1, std::placeholders::_2));
}

void LightControl::CB_timer()
{
    sendCmd();
}

void LightControl::sendCmd(void)
{
    _pubLights->publish(_msgLights);
}

void LightControl::CB_srv(const std::shared_ptr<rover_msgs::srv::LightControl::Request> request,
                          std::shared_ptr<rover_msgs::srv::LightControl::Response> response)
{
    response->success = false;

    if (request->index == rover_msgs::srv::LightControl::Request::LIGHT)
    {
        RCLCPP_INFO(LOGGER, request->enable == true ? "Light enabled": "Light disabled");
        _msgLights.enable[rover_msgs::msg::LightControl::LIGHT] = request->enable;
        response->success = true;
    }

    if (request->index == rover_msgs::srv::LightControl::Request::LIGHT_INFRARED)
    {
        RCLCPP_INFO(LOGGER, request->enable == true ? "Infrared light enabled": "Infrared light disabled");
        _msgLights.enable[rover_msgs::msg::LightControl::LIGHT_INFRARED] = request->enable;
        response->success = true;
    }

    if (response->success == false)
    {
        RCLCPP_WARN(LOGGER, "Error when processing call, no action done");
    }

    this->sendCmd();
}
