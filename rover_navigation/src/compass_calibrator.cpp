#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/compass.hpp"
#include "rover_msgs/srv/compass_calibration.hpp"
#include "rovus_lib/macros.h"

class CompassCalibrator : public rclcpp::Node
{
public:
    CompassCalibrator();
    ~CompassCalibrator() {}

private:
    rclcpp::Publisher<rover_msgs::msg::Compass>::SharedPtr _pub_compass;
    rclcpp::Subscription<rover_msgs::msg::Compass>::SharedPtr _sub_compassRaw;
    rclcpp::Service<rover_msgs::srv::CompassCalibration>::SharedPtr _srv_compassCalib;
    rover_msgs::msg::Compass _msgCompass;

    float _offset = 0.0f;
    float _rawHeading = 0.0f;
    float _calibratedHeading = 0.0f;

    void compassCallback(const rover_msgs::msg::Compass::SharedPtr msgCompass_);
    void CB_srv(const std::shared_ptr<rover_msgs::srv::CompassCalibration::Request> request_, 
                std::shared_ptr<rover_msgs::srv::CompassCalibration::Response> response_);
    void changeHeadingZero();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompassCalibrator>());
    rclcpp::shutdown();
    return 0;
}

CompassCalibrator::CompassCalibrator() : Node("compass_calibrator")
{
    _pub_compass = this->create_publisher<rover_msgs::msg::Compass>("/rover/auxiliary/compass", 1);
    _sub_compassRaw = this->create_subscription<rover_msgs::msg::Compass>("/rover/auxiliary/compass/raw",
                                                                            1,
                                                                            std::bind(&CompassCalibrator::compassCallback, this, std::placeholders::_1));

    _srv_compassCalib = this->create_service<rover_msgs::srv::CompassCalibration>("/rover/auxiliary/compass/calibrate",
                                                                                    std::bind(&CompassCalibrator::CB_srv,
                                                                                    this,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2));
}

void CompassCalibrator::compassCallback(const rover_msgs::msg::Compass::SharedPtr msgCompass_)
{
    _rawHeading = msgCompass_->heading;
    this->changeHeadingZero();

    _msgCompass.heading = _calibratedHeading;
    _msgCompass.pitch = msgCompass_->pitch;
    _pub_compass->publish(_msgCompass);
}

void CompassCalibrator::CB_srv(const std::shared_ptr<rover_msgs::srv::CompassCalibration::Request> request_, 
            std::shared_ptr<rover_msgs::srv::CompassCalibration::Response> response_)
{
    response_->success = false;

    _offset = request_->angle_offset - _rawHeading;

    while (_offset > 180.0f) _offset -= 360.0f;
    while (_offset < -180.0f) _offset += 360.0f;

    response_->success = true;
}

void CompassCalibrator::changeHeadingZero()
{
    _calibratedHeading = _rawHeading + _offset;
    while (_calibratedHeading >= 360.0f) _calibratedHeading -= 360.0f;
    while (_calibratedHeading < 0.0f) _calibratedHeading += 360.0f;
}