#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "rovus_lib/macros.h"
#include "math.h"

#define MAX_SPEED 1.0 // deg/s

using namespace std::chrono_literals;

class Autonomus : public rclcpp::Node
{
public:
    Autonomus();
    ~Autonomus() {}

private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps_rover;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gps_antenna;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_cmd;

    void callbackGPSRover(const rover_msgs::msg::Gps msg);
    void callbackGPSAntenna(const rover_msgs::msg::Gps msg);
    void autonomusCommand();
    float calculateHeading(float lat1, float lon1, float lat2, float lon2);

    rover_msgs::msg::Gps gps_rover;
    rover_msgs::msg::Gps gps_antenna;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Autonomus>());
    rclcpp::shutdown();
    return 0;
}

Autonomus::Autonomus() : Node("autonomus")
{
    _sub_gps_rover = this->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Gps msg)
                                                                     { callbackGPSRover(msg); });

    _sub_gps_antenna = this->create_subscription<rover_msgs::msg::Gps>("/base/antenna/gps/position",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Gps msg)
                                                                     { callbackGPSAntenna(msg); });

    _pub_cmd = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/auto", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&Autonomus::timer_callback, this));
}

void Autonomus::callbackGPSRover(const rover_msgs::msg::Gps msg)
{
    gps_rover = msg;
    autonomusCommand();
}

void Autonomus::callbackGPSAntenna(const rover_msgs::msg::Gps msg)
{
    gps_antenna = msg;
}

void Autonomus::autonomusCommand()
{
    float heading;
    float heading_antenna = gps_antenna.heading;
    if (heading_antenna > 180)
    {
        heading_antenna = heading_antenna - 360;
    }
    float heading_rover = calculateHeading(gps_antenna.latitude, gps_antenna.longitude, gps_rover.latitude, gps_rover.longitude);
    if (heading_antenna > 0)
    {
        heading = heading_rover - heading_antenna;
    }
    else
    {
        heading = (heading_antenna - heading_rover) * (-1);
    }
    //float actual_antenna_heading = gps_antenna.heading;     // verifier que heading est le bon msg a prendre
    //float heading = gps_rover.heading - gps_antenna.heading;    // verifier que heading est le bon msg a prendre
    rover_msgs::msg::AntennaCmd cmd;

    if (heading > 15)
    {
        cmd.status = true;
        cmd.speed = MAX_SPEED;
        _pub_cmd->publish(cmd);
    }
    else if (heading < -15)
    {
        cmd.status = true;
        //cmd.speed = -MAX_SPEED;
        cmd.speed = heading_rover;
        _pub_cmd->publish(cmd);
    }
}

float Autonomus::calculateHeading(float lat1, float lon1, float lat2, float lon2)
{
    float R = 6371e3;
    float PI = 3.14159265359;
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    lon1 = lon1 * PI / 180.0;
    lon2 = lon2 * PI / 180.0;
    
    float X = cos(lat2) * sin(lon2-lon1);
    float Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2-lon1);
    float heading = atan2(X, Y) * 180 / PI;

    return heading;
}
