#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/antenna_cmd.hpp"
#include "rover_msgs/msg/gps.hpp"
#include "rovus_lib/moving_average.hpp"

#define PI 3.14159265359
#define COEFF_NB 10

class Autonomus : public rclcpp::Node
{
public:
    Autonomus();
    ~Autonomus() {}

private:
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gpsRover;
    rclcpp::Subscription<rover_msgs::msg::Gps>::SharedPtr _sub_gpsAntenna;
    rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr _pub_cmd;

    void callbackGPSRover(const rover_msgs::msg::Gps msg_);
    void callbackGPSAntenna(const rover_msgs::msg::Gps msg_);
    void autonomusCommand();
    float calculateHeading(float lat1_, float lon1_, float lat2_, float lon2_);

    rover_msgs::msg::Gps _gpsRover;
    rover_msgs::msg::Gps _gpsAntenna;

    rclcpp::Parameter _paramMaxSpeed;
    rclcpp::Parameter _paramCoeffNb;

    MovingAverage<float, COEFF_NB> _latitudeRover = MovingAverage<float, COEFF_NB>(0.0f);
    MovingAverage<float, COEFF_NB> _longitudeRover = MovingAverage<float, COEFF_NB>(0.0f);

    MovingAverage<float, COEFF_NB> _latitudeAntenna = MovingAverage<float, COEFF_NB>(0.0f);
    MovingAverage<float, COEFF_NB> _longitudeAntenna = MovingAverage<float, COEFF_NB>(0.0f);
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
    _sub_gpsRover = this->create_subscription<rover_msgs::msg::Gps>("/rover/gps/position",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Gps msg_)
                                                                     { callbackGPSRover(msg_); });

    _sub_gpsAntenna = this->create_subscription<rover_msgs::msg::Gps>("/base/antenna/gps/position",
                                                                     1,
                                                                     [this](const rover_msgs::msg::Gps msg_)
                                                                     { callbackGPSAntenna(msg_); });

    _pub_cmd = this->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/in/auto", 1);

    this->declare_parameter<float>("max_speed", PI/2.0f);
    _paramMaxSpeed = this->get_parameter("max_speed");
}

void Autonomus::callbackGPSRover(const rover_msgs::msg::Gps msg_)
{
    _gpsRover.latitude = msg_.latitude;
    _gpsRover.longitude = msg_.longitude;
}

void Autonomus::callbackGPSAntenna(const rover_msgs::msg::Gps msg_)
{
    _gpsAntenna.latitude = msg_.latitude;
    _gpsAntenna.longitude = msg_.longitude;
    _gpsAntenna.heading = msg_.heading;
    
    autonomusCommand();

}

void Autonomus::autonomusCommand()
{
    float relativeHeading;
    float headingAntenna = _gpsAntenna.heading;
    if (headingAntenna > 180.0f)
    {
        headingAntenna = headingAntenna - 360.0f;
    }
    float heading_rover = calculateHeading(_gpsAntenna.latitude, _gpsAntenna.longitude, _gpsRover.latitude, _gpsRover.longitude);
    if (headingAntenna > 0.0f)
    {
        relativeHeading = heading_rover - headingAntenna;
    }
    else
    {
        relativeHeading = (headingAntenna - heading_rover) * (-1);
    }

    rover_msgs::msg::AntennaCmd cmd;

    if (relativeHeading > 15.0f)
    {
        cmd.enable = true;
        cmd.speed = _paramMaxSpeed.as_double();
    }
    else if (relativeHeading < -15.0f)
    {
        cmd.enable = true;
        cmd.speed = -(_paramMaxSpeed.as_double());
    }
    else
    {
        cmd.enable = false;
        cmd.speed = 0.0f;

    }
    RCLCPP_INFO(rclcpp::get_logger("angle"), "angle %f", relativeHeading);
    _pub_cmd->publish(cmd);
}

float Autonomus::calculateHeading(float lat1_, float lon1_, float lat2_, float lon2_)
{
    lat1_ = lat1_ * PI / 180.0f;
    lat2_ = lat2_ * PI / 180.0f;
    lon1_ = lon1_ * PI / 180.0f;
    lon2_ = lon2_ * PI / 180.0f;
    
    float X = cos(lat2_) * sin(lon2_-lon1_);
    float Y = cos(lat1_) * sin(lat2_) - sin(lat1_) * cos(lat2_) * cos(lon2_-lon1_);
    float heading = atan2(X, Y) * 180.0f / PI;

    return heading;
}
