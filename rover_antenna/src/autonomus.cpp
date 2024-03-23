#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "rovus_lib/moving_average.hpp"

#define PI 3.14159265359
#define MAX_SPEED 20.0*PI/180 // rad/s
#define N_AVERAGE 10


using namespace std::chrono_literals;

class Autonomus : public rclcpp::Node
{
public:
    Autonomus();
    ~Autonomus() {}

private:
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

    this->declare_parameter<float>("max_speed", PI/2);
    this->declare_parameter<int16_t>("n_average", 10);

    param_max_speed = this->get_parameter("max_speed");
    param_n_average = this->get_parameter("n_average");
}

void Autonomus::callbackGPSRover(const rover_msgs::msg::Gps msg)
{
    gps_rover.latitude = msg.latitude;
    gps_rover.longitude = msg.longitude;
}

void Autonomus::callbackGPSAntenna(const rover_msgs::msg::Gps msg)
{
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
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    lon1 = lon1 * PI / 180.0;
    lon2 = lon2 * PI / 180.0;
    
    float X = cos(lat2) * sin(lon2-lon1);
    float Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2-lon1);
    float heading = atan2(X, Y) * 180 / PI;

    return heading;
}
