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
#include "rovus_lib/moving_average.hpp"

#define PI 3.14159265359
#define MAX_SPEED 5.0*PI/180 // rad/s
#define N_AVERAGE 10


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
    rover_msgs::msg::Gps gps_rover_SMA[10];
    rover_msgs::msg::Gps gps_antenna_SMA[10];
    bool first_msgs_rover = true;
    int count_rover = 0;
    bool first_msgs_antenna = true;
    int count_antenna = 0;

    rclcpp::Parameter param_max_speed;
    rclcpp::Parameter param_n_average;

    MovingAverage<float, N_AVERAGE> latitude_rover = MovingAverage<float, N_AVERAGE>(0.0);
    MovingAverage<float, N_AVERAGE> longitude_rover = MovingAverage<float, N_AVERAGE>(0.0);

    MovingAverage<float, N_AVERAGE> latitude_antenna = MovingAverage<float, N_AVERAGE>(0.0);
    MovingAverage<float, N_AVERAGE> longitude_antenna = MovingAverage<float, N_AVERAGE>(0.0);
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

    this->declare_parameter<float>("max_speed", PI/2);
    this->declare_parameter<int16_t>("n_average", 10);

    param_max_speed = this->get_parameter("max_speed");
    param_n_average = this->get_parameter("n_average");
}

void Autonomus::callbackGPSRover(const rover_msgs::msg::Gps msg)
{
    // float rover_lat;
    // float rover_long;
    // if (count_rover < N_SMA && first_msgs_rover)
    // {
    //     gps_rover_SMA[count_rover].latitude = msg.latitude;
    //     gps_rover_SMA[count_rover].longitude = msg.longitude;
    //     count_rover++;
    // }
    // else if (count_rover == N_SMA && !first_msgs_rover)
    // {
    //     for (int i=0; i<N_SMA-1; i++)
    //     {
    //         gps_rover_SMA[i].latitude = gps_rover_SMA[i+1].latitude;
    //         gps_rover_SMA[i].longitude = gps_rover_SMA[i+1].longitude;
    //     }
    // }
    // if (count_rover == 10)
    // {
    //     first_msgs_rover = false;
    // }
    // if (!first_msgs_rover)
    // {
    //     rover_lat = 0;
    //     rover_long = 0;
    //     for (int i=0; i<N_SMA; i++)
    //     {
    //         rover_lat = rover_lat + gps_rover_SMA[i].latitude;
    //         rover_long = rover_long + gps_rover_SMA[i].longitude;
    //     }
    //     rover_lat = rover_lat/N_SMA;
    //     rover_long = rover_long/N_SMA;
    //     gps_rover.latitude = rover_lat;
    //     gps_rover.longitude = rover_long;
    //     RCLCPP_INFO(rclcpp::get_logger("lat"), "Latitude rover %f", rover_lat);
    //     autonomusCommand();
    // }
    // latitude_rover.addValue(msg.latitude);
    // longitude_rover.addValue(msg.longitude);

    // gps_rover.latitude = latitude_rover.getAverage();
    // gps_rover.longitude = longitude_rover.getAverage();

    gps_rover.latitude = msg.latitude;
    gps_rover.longitude = msg.longitude;
    
}

void Autonomus::callbackGPSAntenna(const rover_msgs::msg::Gps msg)
{
    // float antenna_lat;
    // float antenna_long;
    // if (count_antenna < N_SMA && first_msgs_antenna)
    // {
    //     gps_antenna_SMA[count_antenna].latitude = msg.latitude;
    //     gps_antenna_SMA[count_antenna].longitude = msg.longitude;
    //     count_antenna++;
    // }
    // else if (count_antenna == N_SMA && !first_msgs_antenna)
    // {
    //     for (int i=0; i<N_SMA-1; i++)
    //     {
    //         gps_antenna_SMA[i].latitude = gps_antenna_SMA[i+1].latitude;
    //         gps_antenna_SMA[i].longitude = gps_antenna_SMA[i+1].longitude;
    //     }
    // }
    // if (count_antenna == 10)
    // {
    //     first_msgs_antenna = false;
    // }
    // if (!first_msgs_antenna)
    // {
    //     antenna_lat = 0;
    //     antenna_long = 0;
    //     for (int i=0; i<N_SMA; i++)
    //     {
    //         antenna_lat = antenna_lat + gps_antenna_SMA[i].latitude;
    //         antenna_long = antenna_long + gps_antenna_SMA[i].longitude;
    //     }
    //     antenna_lat = antenna_lat/N_SMA;
    //     antenna_long = antenna_long/N_SMA;
    //     gps_antenna.latitude = antenna_lat;
    //     gps_antenna.longitude = antenna_long;
    //     RCLCPP_INFO(rclcpp::get_logger("lat"), "Latitude antenna %f", antenna_lat);
    // }

    // latitude_antenna.addValue(msg.latitude);
    // longitude_antenna.addValue(msg.longitude);

    // gps_antenna.latitude = latitude_antenna.getAverage();
    // gps_antenna.longitude = longitude_antenna.getAverage();

    gps_antenna.latitude = msg.latitude;
    gps_antenna.longitude = msg.longitude;
    gps_antenna.heading = msg.heading;
    
    autonomusCommand();

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

    rover_msgs::msg::AntennaCmd cmd;

    if (heading > 15.0)
    {
        cmd.status = true;
        cmd.speed = MAX_SPEED;
    }
    else if (heading < -15.0)
    {
        cmd.status = true;
        cmd.speed = -MAX_SPEED;
    }
    else
    {
        cmd.status = false;
        cmd.speed = 0.0;

    }
    RCLCPP_INFO(rclcpp::get_logger("angle"), "angle %f", heading);
    _pub_cmd->publish(cmd);
}

float Autonomus::calculateHeading(float lat1, float lon1, float lat2, float lon2)
{
    //float R = 6371e3;
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    lon1 = lon1 * PI / 180.0;
    lon2 = lon2 * PI / 180.0;
    
    float X = cos(lat2) * sin(lon2-lon1);
    float Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2-lon1);
    float heading = atan2(X, Y) * 180 / PI;

    return heading;
}
