#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/compass_calibrated.hpp"

class CompassCalibrator : public rclcpp::Node
{
public:
    CompassCalibrator();
    ~CompassCalibrator() {}

private:
    rclcpp::Publisher<rover_msgs::msg::CompassCalibrated>::SharedPtr _pubCompassCalibrated;
    rclcpp::TimerBase::SharedPtr _timerPub;

    float _offset = 0.0f;
    float _rawHeading = 0.0f;
    float _calibratedHeading = 0.0f;

    void compassCallback(const rover_msgs::msg::Compass::SharedPtr msgCompass_);
    void CB_srv(const std::shared_ptr<rover_msgs::srv::CompassCalibration::Request> request, 
                std::shared_ptr<rover_msgs::srv::CompassCalibration::Response> response);
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
    _pubCompassCalibrated = this->create_publisher<rover_msgs::msg::CompassCalibrated>("/rover/auxiliary/compass/orientation", 1);
    _timerPub = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CompassCalibrator::CB_timer, this));
}

void CompassCalibrator::CB_timer()
{
    sendCmd();
}

void CompassCalibrator::sendCmd(void)
{
    response->success = false;

    _offset = request->angle_offset - _rawHeading;

    while (_offset > 180.0f) _offset -= 360.0f;
    while (_offset < -180.0f) _offset += 360.0f;

    response->success = true;
}

void CompassCalibrator::changeHeadingZero()
{
    _calibratedHeading = _rawHeading + _offset;
    while (_calibratedHeading >= 360.0f) _calibratedHeading -= 360.0f;
    while (_calibratedHeading < 0.0f) _calibratedHeading += 360.0f;
}