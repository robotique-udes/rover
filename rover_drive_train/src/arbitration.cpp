#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rover_msgs/msg/joy.hpp"
#include "rover_msgs/msg/propulsion_motor.hpp"
#include "rover_msgs/msg/joy_demux_status.hpp"
#include "rover_msgs/srv/drive_train_arbitration.hpp"

//Class definition
class Arbitration : public rclcpp::Node
{
    public:
        Arbitration();

        enum DemuxDestination
        {
            security = (int8_t)rover_msgs::srv::DriveTrainArbitration_Request::SECURITY,
            teleop = (int8_t)rover_msgs::srv::DriveTrainArbitration_Request::TELEOP,
            autonomous = (int8_t)rover_msgs::srv::DriveTrainArbitration_Request::AUTONOMUS
        };

    private:
        rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_motor_cmd;
        rclcpp::Subscription<rover_msgs::msg::PropulsionMotor>::SharedPtr _sub_security;

        rclcpp::Service<rover_msgs::srv::DriveTrainArbitration>::SharedPtr _srv_control_demux;

        void callbackDemux(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request,
                                std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response);
};

Arbitration::Arbitration() : Node("arbitration")
{
    _srv_control_demux = this->create_service<rover_msgs::srv::DriveTrainArbitration>("demux_control_cmd", std::bind(&Arbitration::callbackDemux, this, std::placeholders::_1, std::placeholders::_2));
}

void Arbitration::callbackDemux(const std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Request> request,
                                      std::shared_ptr<rover_msgs::srv::DriveTrainArbitration::Response> response)
{
  
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arbitration>());
    rclcpp::shutdown();
    
    return 0;
}
