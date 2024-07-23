

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/compass_calibrated.hpp"

class CompassCalibrator: public rclcpp::Node
{
public:
    CompassCalibrator();
    ~CompassCalibrator() {}

private:
    rclcpp::Publisher<rover_msgs::msg::CompassCalibrated>::SharedPtr _pubCompassCalibrated;
    rclcpp::TimerBase::SharedPtr _timerPub;

    rover_msgs::msg::CompassCalibrated _msgCompassCalibrated;
    void CB_timer(void);
    void sendCmd(void);
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompassCalibrator>());
  rclcpp::shutdown();
  return 0;
}

CompassCalibrator::CompassCalibrator() : Node("compass_calibrator")
{
    _pubCompassCalibrated = this->create_publisher<rover_msgs::msg::CompassCalibrated>("/rover/auxiliary/orientation", 1);
    _timerPub = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CompassCalibrator::CB_timer, this));

}

void CompassCalibrator::CB_timer()
{
    sendCmd();
}

void CompassCalibrator::sendCmd(void)
{
    _pubCompassCalibrated->publish(_msgCompassCalibrated);
}